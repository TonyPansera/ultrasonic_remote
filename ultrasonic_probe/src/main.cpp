// GPIO32 ultrasonic signal probe for the LilyGO T-Display v1.1.
//
// GPIO32 (ADC1 channel 4) is sampled through the I2S built-in-ADC DMA path (~1 MS/s, far beyond
// what analogRead() can do), shown live on the TFT (triggered waveform, frequency, amplitude),
// and streamed over Serial on request for host-side analysis (tools/analyze_signal.py).
//
// At boot, before the ADC takes over the pin, two independent references are measured:
//   - a PCNT hardware edge count on GPIO32 (exact frequency, if the signal crosses the logic threshold)
//   - an analogRead() average (to cross-check the DC level / data polarity of the DMA path)
//
// Serial protocol (921600 baud, one command per line, replies are "KEY k=v ..." lines):
//   info        -> INFO line (rates, boot measurements) + CAL line (raw code -> mV table)
//   rate <Hz>   -> restart sampling at <Hz>; replies RATE ok=.. cfg=.. meas=..
//   cap <n>     -> CAP n=.. rate=.., then n raw 16-bit words as hex (64 per line), then END sum=..
//   pull <none|up|down|both> -> internal ~45 kOhm pull(s) on the sampled pin (connection test); PULL mode=..
//                  A driven signal barely moves; a floating (unconnected) pin follows the pull.
//                  "both" holds an unconnected pin at mid-rail through ~22 kOhm.
//   src <32|33> -> sample GPIO32 (signal) or GPIO33 (unconnected control pin); SRC gpio=.. ch=..
//   disp <on|off> -> wake / sleep the ST7789 + backlight (its charge pump is a ~41 kHz noise source); DISP on=..
// Raw word layout: bits 15..12 = ADC channel (4 for GPIO32), bits 11..0 = 12-bit sample.
// The I2S FIFO returns 16-bit mono samples with each pair swapped; consumers must un-swap.

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include <driver/pcnt.h>
#include <driver/rtc_io.h>
#include <esp_adc_cal.h>

constexpr int PIN_SIGNAL = 32;
constexpr adc1_channel_t SIGNAL_CHANNEL = ADC1_CHANNEL_4;  // GPIO32
constexpr int PIN_CONTROL = 33;
constexpr adc1_channel_t CONTROL_CHANNEL = ADC1_CHANNEL_5;  // GPIO33, nothing connected
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr uint32_t DEFAULT_RATE = 1000000;
constexpr int DMA_BUF_LEN = 1024;  // samples per DMA buffer
constexpr int DMA_BUF_COUNT = 8;
constexpr size_t CAPTURE_MAX = 40960;  // 80 KB, ~41 ms at 1 MS/s
constexpr size_t LIVE_SAMPLES = 4096;
constexpr int PCNT_GATES = 4;
constexpr uint32_t PCNT_GATE_MS = 250;

// Live verdict thresholds (the host script applies the same ones)
constexpr float BAND_LOW_HZ = 38000;
constexpr float BAND_HIGH_HZ = 42000;
constexpr float MIN_VPP_MV = 20;
constexpr float MAX_CLIP_FRACTION = 0.001f;

constexpr int SCOPE_Y = 70;
constexpr int SCOPE_H = 65;

TFT_eSPI tft;
TFT_eSprite scope(&tft);
esp_adc_cal_characteristics_t adcChars;
esp_adc_cal_value_t calType;

uint16_t *captureBuf = nullptr;
uint16_t samples[LIVE_SAMPLES];  // un-swapped 12-bit codes of the last live capture
uint32_t rateCfg = 0;
const char *pullMode = "none";
int sourcePin = PIN_SIGNAL;
bool displayOn = true;
adc1_channel_t sourceChannel = SIGNAL_CHANNEL;
float rateMeas = 0;

struct BootStats {
  float pcntHz;
  int16_t pcntCounts[PCNT_GATES];
  float oneshotMean;
  int oneshotMin;
  int oneshotMax;
} boot;

struct LiveStats {
  float freqHz;
  float vppMv;
  float dcMv;
  float clipFraction;
  float validFraction;
  int triggerIndex;
  uint16_t minCode;
  uint16_t maxCode;
} live;

// ---------- boot-time reference measurements ----------

void measurePcnt() {
  pcnt_config_t pc = {};
  pc.pulse_gpio_num = PIN_SIGNAL;
  pc.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pc.lctrl_mode = PCNT_MODE_KEEP;
  pc.hctrl_mode = PCNT_MODE_KEEP;
  pc.pos_mode = PCNT_COUNT_INC;
  pc.neg_mode = PCNT_COUNT_DIS;
  pc.counter_h_lim = 32767;
  pc.counter_l_lim = -1;
  pc.unit = PCNT_UNIT_0;
  pc.channel = PCNT_CHANNEL_0;
  pcnt_unit_config(&pc);
  // The driver enables the internal pull-up; keep the analog source unloaded.
  gpio_set_pull_mode((gpio_num_t)PIN_SIGNAL, GPIO_FLOATING);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);  // ignore glitches < 1.25 us (a 40 kHz half period is 12.5 us)
  pcnt_filter_enable(PCNT_UNIT_0);

  uint32_t totalCount = 0;
  uint32_t totalUs = 0;
  for (int g = 0; g < PCNT_GATES; g++) {
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    uint32_t t0 = micros();
    pcnt_counter_resume(PCNT_UNIT_0);
    delay(PCNT_GATE_MS);
    pcnt_counter_pause(PCNT_UNIT_0);
    uint32_t dt = micros() - t0;
    pcnt_get_counter_value(PCNT_UNIT_0, &boot.pcntCounts[g]);
    totalCount += boot.pcntCounts[g];
    totalUs += dt;
  }
  boot.pcntHz = totalCount * 1e6f / totalUs;
}

void measureOneshot() {
  analogSetPinAttenuation(PIN_SIGNAL, ADC_11db);  // same setting as ADC_ATTEN_DB_12
  const int n = 2000;
  long sum = 0;
  boot.oneshotMin = 4095;
  boot.oneshotMax = 0;
  for (int i = 0; i < n; i++) {
    int v = analogRead(PIN_SIGNAL);
    sum += v;
    boot.oneshotMin = min(boot.oneshotMin, v);
    boot.oneshotMax = max(boot.oneshotMax, v);
  }
  boot.oneshotMean = (float)sum / n;
}

// ---------- I2S ADC sampling ----------

void readChunk(uint16_t *dst) {
  size_t got = 0;
  i2s_read(I2S_PORT, dst, DMA_BUF_LEN * sizeof(uint16_t), &got, portMAX_DELAY);
}

// Drop whatever accumulated in the DMA queue while nobody was reading.
void flushDma() {
  static uint16_t scratch[DMA_BUF_LEN];
  for (int i = 0; i < DMA_BUF_COUNT + 1; i++) readChunk(scratch);
}

// Real sample rate = samples delivered per second of CPU time. The I2S clock dividers do not
// always hit the requested rate exactly, so all frequency math uses this measured value.
float measureRate() {
  static uint16_t scratch[DMA_BUF_LEN];
  const int chunks = 64;
  flushDma();
  uint32_t t0 = micros();
  for (int i = 0; i < chunks; i++) readChunk(scratch);
  uint32_t dt = micros() - t0;
  return chunks * (float)DMA_BUF_LEN * 1e6f / dt;
}

void applyPull(const char *mode) {
  gpio_num_t pin = (gpio_num_t)sourcePin;
  if (strcmp(mode, "both") == 0) {
    rtc_gpio_pullup_en(pin);
    rtc_gpio_pulldown_en(pin);
    pullMode = "both";
  } else if (strcmp(mode, "up") == 0) {
    rtc_gpio_pulldown_dis(pin);
    rtc_gpio_pullup_en(pin);
    pullMode = "up";
  } else if (strcmp(mode, "down") == 0) {
    rtc_gpio_pullup_dis(pin);
    rtc_gpio_pulldown_en(pin);
    pullMode = "down";
  } else {
    rtc_gpio_pullup_dis(pin);
    rtc_gpio_pulldown_dis(pin);
    pullMode = "none";
  }
}

bool startAdc(uint32_t rate) {
  if (rateCfg) {
    i2s_adc_disable(I2S_PORT);
    i2s_driver_uninstall(I2S_PORT);
    rateCfg = 0;
  }
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  cfg.sample_rate = rate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = DMA_BUF_COUNT;
  cfg.dma_buf_len = DMA_BUF_LEN;
  cfg.use_apll = false;
  if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) return false;
  i2s_set_adc_mode(ADC_UNIT_1, sourceChannel);
  adc1_config_channel_atten(sourceChannel, ADC_ATTEN_DB_12);
  i2s_adc_enable(I2S_PORT);
  applyPull(pullMode);  // the ADC pad (re)init in the calls above clears any pull
  rateCfg = rate;
  rateMeas = measureRate();
  return true;
}

void capture(size_t n) {
  flushDma();
  size_t got = 0;
  i2s_read(I2S_PORT, captureBuf, n * sizeof(uint16_t), &got, portMAX_DELAY);
}

// ---------- live analysis + display ----------

uint32_t codeToMv(uint16_t code) { return esp_adc_cal_raw_to_voltage(code, &adcChars); }

void analyzeLive() {
  capture(LIVE_SAMPLES);

  size_t valid = 0;
  for (size_t i = 0; i < LIVE_SAMPLES; i++) {
    uint16_t w = captureBuf[i ^ 1];  // un-swap sample pairs
    valid += (w >> 12) == sourceChannel;
    samples[i] = w & 0x0FFF;
  }

  uint32_t sum = 0;
  size_t clipped = 0;
  live.minCode = 4095;
  live.maxCode = 0;
  for (size_t i = 0; i < LIVE_SAMPLES; i++) {
    uint16_t v = samples[i];
    sum += v;
    clipped += (v == 0 || v == 4095);
    live.minCode = min(live.minCode, v);
    live.maxCode = max(live.maxCode, v);
  }
  float mean = (float)sum / LIVE_SAMPLES;
  live.validFraction = (float)valid / LIVE_SAMPLES;
  live.clipFraction = (float)clipped / LIVE_SAMPLES;
  live.vppMv = (float)codeToMv(live.maxCode) - (float)codeToMv(live.minCode);
  live.dcMv = codeToMv((uint16_t)(mean + 0.5f));

  // Rising crossings of the mean, re-armed only after dropping below mean - hysteresis.
  float hyst = 0.1f * (live.maxCode - live.minCode);
  bool armed = false;
  int crossings = 0;
  float firstPos = 0, lastPos = 0;
  live.triggerIndex = -1;
  for (size_t i = 1; i < LIVE_SAMPLES; i++) {
    if (samples[i] < mean - hyst) {
      armed = true;
    } else if (armed && samples[i - 1] < mean && samples[i] >= mean) {
      float pos = (i - 1) + (mean - samples[i - 1]) / (float)(samples[i] - samples[i - 1]);
      if (crossings == 0) {
        firstPos = pos;
        live.triggerIndex = i;
      }
      lastPos = pos;
      crossings++;
      armed = false;
    }
  }
  live.freqHz = (crossings >= 2 && hyst > 2) ? rateMeas * (crossings - 1) / (lastPos - firstPos) : 0;
}

void drawLive() {
  bool inBand = live.freqHz >= BAND_LOW_HZ && live.freqHz <= BAND_HIGH_HZ;
  const char *status;
  uint16_t statusColor;
  if (live.validFraction < 0.99f) {
    status = "ADC ERR";
    statusColor = TFT_RED;
  } else if (live.vppMv < MIN_VPP_MV) {
    status = "NO SIGNAL";
    statusColor = TFT_RED;
  } else if (live.clipFraction > MAX_CLIP_FRACTION) {
    status = "CLIPPING";
    statusColor = TFT_ORANGE;
  } else if (!inBand) {
    status = "OFF BAND";
    statusColor = TFT_ORANGE;
  } else {
    status = "OK";
    statusColor = TFT_GREEN;
  }

  char line[40];
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(0);
  snprintf(line, sizeof(line), "GPIO%d ultrasonic", sourcePin);
  tft.drawString(line, 4, 2, 2);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(statusColor, TFT_BLACK);
  tft.setTextPadding(90);
  tft.drawString(status, tft.width() - 4, 2, 2);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextPadding(tft.width() - 8);
  if (live.freqHz > 0) {
    snprintf(line, sizeof(line), "%.3f kHz", live.freqHz / 1000);
  } else {
    snprintf(line, sizeof(line), "--- kHz");
  }
  tft.drawString(line, 4, 20, 4);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  snprintf(line, sizeof(line), "Vpp %.0f mV  DC %.0f mV", live.vppMv, live.dcMv);
  tft.drawString(line, 4, 50, 2);

  // Triggered waveform, one sample per pixel (~240 us at 1 MS/s), auto-scaled.
  scope.fillSprite(TFT_BLACK);
  for (int x = 0; x < scope.width(); x += 24) scope.drawFastVLine(x, 0, SCOPE_H, 0x18E3);
  scope.drawFastHLine(0, SCOPE_H / 2, scope.width(), 0x18E3);
  int start = live.triggerIndex > 0 ? live.triggerIndex : 0;
  int span = max(1, live.maxCode - live.minCode);
  int prevY = -1;
  for (int x = 0; x < scope.width() && start + x < (int)LIVE_SAMPLES; x++) {
    int y = SCOPE_H - 2 - (samples[start + x] - live.minCode) * (SCOPE_H - 4) / span;
    if (prevY >= 0) scope.drawLine(x - 1, prevY, x, y, TFT_GREEN);
    prevY = y;
  }
  scope.pushSprite(0, SCOPE_Y);
}

// ---------- serial protocol ----------

void printInfo() {
  const char *cal = calType == ESP_ADC_CAL_VAL_EFUSE_TP   ? "two_point"
                    : calType == ESP_ADC_CAL_VAL_EFUSE_VREF ? "efuse_vref"
                                                            : "default_vref";
  Serial.printf("INFO rate_cfg=%lu rate_meas=%.1f cap_max=%u pcnt_hz=%.1f pcnt_gate_ms=%lu pcnt_counts=",
                (unsigned long)rateCfg, rateMeas, (unsigned)CAPTURE_MAX, boot.pcntHz,
                (unsigned long)PCNT_GATE_MS);
  for (int g = 0; g < PCNT_GATES; g++) Serial.printf(g ? ",%d" : "%d", boot.pcntCounts[g]);
  Serial.printf(" oneshot_mean=%.1f oneshot_min=%d oneshot_max=%d cal=%s vref=%lu pull=%s src=%d\n",
                boot.oneshotMean, boot.oneshotMin, boot.oneshotMax, cal, (unsigned long)adcChars.vref, pullMode,
                sourcePin);
  Serial.print("CAL");
  for (int raw = 0; raw <= 4096; raw += 64) {
    int code = min(raw, 4095);
    Serial.printf(" %d:%lu", code, (unsigned long)codeToMv(code));
  }
  Serial.println();
}

void dumpCapture(size_t n) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  static char line[64 * 4 + 1];
  uint32_t sum = 0;
  Serial.printf("CAP n=%u rate=%.1f ch=%d\n", (unsigned)n, rateMeas, sourceChannel);
  for (size_t i = 0; i < n; i += 64) {
    size_t m = min((size_t)64, n - i);
    char *p = line;
    for (size_t j = 0; j < m; j++) {
      uint16_t w = captureBuf[i + j];
      sum += w;
      *p++ = HEX_DIGITS[(w >> 12) & 0xF];
      *p++ = HEX_DIGITS[(w >> 8) & 0xF];
      *p++ = HEX_DIGITS[(w >> 4) & 0xF];
      *p++ = HEX_DIGITS[w & 0xF];
    }
    *p++ = '\n';
    Serial.write(line, p - line);
  }
  Serial.printf("END sum=%lu\n", (unsigned long)sum);
}

void handleCommand(char *cmd) {
  if (strcmp(cmd, "info") == 0) {
    printInfo();
  } else if (strncmp(cmd, "rate ", 5) == 0) {
    uint32_t previous = rateCfg;
    uint32_t requested = strtoul(cmd + 5, nullptr, 10);
    bool ok = requested >= 10000 && requested <= 2000000 && startAdc(requested);
    if (!ok) startAdc(previous ? previous : DEFAULT_RATE);
    Serial.printf("RATE ok=%d cfg=%lu meas=%.1f\n", ok, (unsigned long)rateCfg, rateMeas);
  } else if (strncmp(cmd, "pull ", 5) == 0) {
    applyPull(cmd + 5);
    Serial.printf("PULL mode=%s\n", pullMode);
  } else if (strncmp(cmd, "disp ", 5) == 0) {
    displayOn = strcmp(cmd + 5, "off") != 0;
    if (displayOn) {
      tft.writecommand(ST7789_SLPOUT);
      delay(120);
      tft.writecommand(ST7789_DISPON);
      digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
    } else {
      digitalWrite(TFT_BL, !TFT_BACKLIGHT_ON);
      tft.writecommand(ST7789_DISPOFF);
      tft.writecommand(ST7789_SLPIN);  // also stops the panel's internal charge pumps
    }
    delay(150);
    Serial.printf("DISP on=%d\n", displayOn);
  } else if (strncmp(cmd, "src ", 4) == 0) {
    int gpio = atoi(cmd + 4);
    if (gpio == PIN_SIGNAL || gpio == PIN_CONTROL) {
      applyPull("none");  // release the pin we are leaving
      sourcePin = gpio;
      sourceChannel = gpio == PIN_SIGNAL ? SIGNAL_CHANNEL : CONTROL_CHANNEL;
      startAdc(rateCfg ? rateCfg : DEFAULT_RATE);
    }
    Serial.printf("SRC gpio=%d ch=%d\n", sourcePin, sourceChannel);
  } else if (strncmp(cmd, "cap ", 4) == 0) {
    size_t n = strtoul(cmd + 4, nullptr, 10);
    n = constrain(n, (size_t)DMA_BUF_LEN, CAPTURE_MAX) & ~(size_t)(DMA_BUF_LEN - 1);
    capture(n);
    dumpCapture(n);
  } else if (cmd[0]) {
    Serial.printf("ERR unknown command '%s'\n", cmd);
  }
}

void pollSerial() {
  static char buf[48];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = 0;
      handleCommand(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void setup() {
  Serial.begin(921600);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Measuring GPIO32...", 4, 4, 2);

  calType = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adcChars);
  // analogRead first: the PCNT driver briefly enables a pull-up, which would precharge a floating pin.
  measureOneshot();
  measurePcnt();

  captureBuf = (uint16_t *)heap_caps_malloc(CAPTURE_MAX * sizeof(uint16_t), MALLOC_CAP_8BIT);
  scope.createSprite(tft.width(), SCOPE_H);
  if (!captureBuf || !startAdc(DEFAULT_RATE)) {
    Serial.println("ERR init failed");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("INIT FAILED", 4, 24, 4);
    while (true) delay(1000);
  }

  tft.fillScreen(TFT_BLACK);
  printInfo();
  Serial.println("READY");
}

void loop() {
  static uint32_t lastLive = 0;
  static uint32_t lastPrint = 0;
  pollSerial();
  if (millis() - lastLive >= 200) {
    lastLive = millis();
    analyzeLive();
    if (displayOn) drawLive();
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.printf("LIVE f=%.1f vpp_mv=%.0f dc_mv=%.0f clip=%.4f valid=%.3f\n", live.freqHz, live.vppMv,
                    live.dcMv, live.clipFraction, live.validFraction);
    }
  }
}
