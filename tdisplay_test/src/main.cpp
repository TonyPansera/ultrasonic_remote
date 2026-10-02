// LilyGO T-Display v1.1 hardware test.
// - Color sweep at boot (check for dead pixels / wrong color order)
// - Chip info, uptime and supply voltage on screen and on Serial
// - Live state of the two user buttons (GPIO0 and GPIO35)

#include <Arduino.h>
#include <TFT_eSPI.h>

constexpr int PIN_BTN_LEFT = 0;
constexpr int PIN_BTN_RIGHT = 35;  // input-only pin, external pull-up on the board
constexpr int PIN_ADC_EN = 14;     // enables the battery/USB voltage divider
constexpr int PIN_BAT_ADC = 34;

constexpr int BTN_W = 70;
constexpr int BTN_H = 18;
constexpr int BTN_Y = 114;

TFT_eSPI tft;

String macString() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           (uint8_t)(mac), (uint8_t)(mac >> 8), (uint8_t)(mac >> 16),
           (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
  return String(buf);
}

// The divider on GPIO34 halves the voltage.
float readSupplyVoltage() {
  return analogReadMilliVolts(PIN_BAT_ADC) * 2 / 1000.0f;
}

void colorSweep() {
  const uint16_t colors[] = {TFT_RED, TFT_GREEN, TFT_BLUE, TFT_WHITE};
  for (uint16_t c : colors) {
    tft.fillScreen(c);
    delay(500);
  }
}

void drawStaticInfo() {
  char line[48];

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Hello T-Display!", 8, 4, 4);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  snprintf(line, sizeof(line), "%s rev%d  %lu MHz", ESP.getChipModel(),
           ESP.getChipRevision(), (unsigned long)ESP.getCpuFreqMHz());
  tft.drawString(line, 8, 34, 2);
  snprintf(line, sizeof(line), "Flash %lu MB",
           (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)));
  tft.drawString(line, 8, 50, 2);
  tft.drawString("MAC " + macString(), 8, 66, 2);
}

void drawButton(int x, const char *label, bool pressed) {
  uint16_t fill = pressed ? TFT_GREEN : TFT_DARKGREY;
  tft.fillRoundRect(x, BTN_Y, BTN_W, BTN_H, 4, fill);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_BLACK, fill);
  tft.drawString(label, x + BTN_W / 2, BTN_Y + BTN_H / 2, 2);
}

void drawDynamicInfo(unsigned long uptimeS, float volts) {
  char line[32];
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(160);  // erase the previous, possibly longer, value
  snprintf(line, sizeof(line), "Uptime %lus", uptimeS);
  tft.drawString(line, 8, 82, 2);
  snprintf(line, sizeof(line), "Supply %.2f V", volts);
  tft.drawString(line, 8, 98, 2);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT);
  pinMode(PIN_ADC_EN, OUTPUT);
  digitalWrite(PIN_ADC_EN, HIGH);

  tft.init();
  tft.setRotation(1);  // landscape, 240x135
  colorSweep();
  drawStaticInfo();

  Serial.println();
  Serial.println("=== T-Display test ===");
  Serial.printf("Chip: %s rev%d, %d cores, %lu MHz\n", ESP.getChipModel(),
                ESP.getChipRevision(), ESP.getChipCores(),
                (unsigned long)ESP.getCpuFreqMHz());
  Serial.printf("Flash: %lu MB, PSRAM: %lu B, heap free: %lu B\n",
                (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)),
                (unsigned long)ESP.getPsramSize(), (unsigned long)ESP.getFreeHeap());
  Serial.printf("MAC: %s\n", macString().c_str());
  Serial.printf("Display: %dx%d, rotation %d\n", tft.width(), tft.height(),
                tft.getRotation());
}

void loop() {
  static int lastLeft = -1;
  static int lastRight = -1;
  static unsigned long lastTick = 0;

  bool left = digitalRead(PIN_BTN_LEFT) == LOW;
  bool right = digitalRead(PIN_BTN_RIGHT) == LOW;
  if (left != lastLeft) {
    drawButton(8, "GPIO0", left);
    if (lastLeft != -1) Serial.printf("GPIO0 %s\n", left ? "pressed" : "released");
    lastLeft = left;
  }
  if (right != lastRight) {
    drawButton(tft.width() - 8 - BTN_W, "GPIO35", right);
    if (lastRight != -1) Serial.printf("GPIO35 %s\n", right ? "pressed" : "released");
    lastRight = right;
  }

  unsigned long now = millis();
  if (now - lastTick >= 1000) {
    lastTick = now;
    unsigned long uptimeS = now / 1000;
    float volts = readSupplyVoltage();
    drawDynamicInfo(uptimeS, volts);
    Serial.printf("uptime=%lus supply=%.2fV btn0=%d btn35=%d\n", uptimeS, volts,
                  left, right);
  }

  delay(20);
}
