# ultrasonic_probe: GPIO32 40 kHz signal checker

Firmware and a host-side analysis script that check whether GPIO32 of the T-Display carries a usable ~40 kHz ultrasonic signal.

- **Latest result:** [results/REPORT.md](results/REPORT.md), with graphs in `results/` and raw captures in `results/data/`.
- **2026-10-02 verdict: FAIL.** GPIO32 sits permanently at or above the ADC full scale (≥ 3.15 V), held high through about 5 kΩ. No 40 kHz signal is measurable. The report lists the hardware checks to do.

## How it works

**Firmware** (`src/main.cpp`):
- Samples GPIO32 (ADC1 channel 4) through the ESP32's I2S built-in-ADC DMA path. The true conversion rate is about **249 kS/s**, or 6.2 samples per 40 kHz period. `analogRead()` would manage only ~20 kS/s.
- At boot, before the ADC takes over the pin, it records two independent references:
  - a hardware edge count (PCNT) on GPIO32 over 1 s, which gives the exact frequency if the signal crosses the logic threshold;
  - an `analogRead()` average.
- The screen shows live status, frequency, Vpp, DC level and a triggered waveform.
  - Status values: `OK`, `NO SIGNAL`, `CLIPPING`, `OFF BAND`, `ADC ERR`.
  - It refreshes about every 200 ms.

**Host script** (`tools/analyze_signal.py`) drives the firmware over serial and runs five steps:
1. **Connection test:** GPIO32 with no pull, the internal pull-down, then the internal pull-up. A driven line barely moves; a floating pin follows the pull.
2. **Sampler sweep:** finds the real ADC rate. The DMA stream repeats each conversion 1×/2×/4×/8× depending on the configured I2S rate.
3. **Long captures** (164 ms each): as wired, and with the pull-down if "as wired" is pinned to a rail.
4. **Trend:** 20 short captures over about 25 s.
5. **Interference controls:** GPIO33 (nothing connected, held at mid-rail) and GPIO32, each with the display awake and asleep. These separate board or supply noise from the real signal.

It then writes `results/`:
- figures `01`–`08`
- `REPORT.md`, with the verdict and hardware checklist
- `summary.json`
- `data/*.npz`

Pass criteria (as wired):
- under 0.1 % of samples clipped
- Vpp ≥ 20 mV
- a tone ≥ 15 dB above the local noise floor between 38 and 42 kHz

## Usage

```bash
cd ~/esp32/ultrasonic_probe
pio run -t upload                      # flash the firmware (board attached as /dev/ttyACM0)
uv run tools/analyze_signal.py         # ~3 min: measure, plot, write results/
uv run tools/analyze_signal.py --trend 5 --captures 1   # quicker run
```

`uv` installs numpy, scipy, matplotlib and pyserial automatically; the dependencies are declared inside the script. Opening the serial port resets the board, so the script waits for `READY` first.

For a quick look without the script, use `pio device monitor` (921600 baud). The firmware prints one `LIVE f=... vpp_mv=... dc_mv=... clip=...` line per second.

## Serial protocol (921600 baud, one command per line)

| Command | Reply |
|---|---|
| `info` | `INFO rate_cfg=.. rate_meas=.. pcnt_hz=.. pcnt_counts=.. oneshot_mean=.. cal=.. pull=.. src=..`, then a `CAL code:mV ...` table |
| `rate <Hz>` | `RATE ok=.. cfg=.. meas=..` (restarts the sampler; `meas` is the DMA rate, see the repeat factor above) |
| `cap <n>` | `CAP n=.. rate=.. ch=..`, then n raw 16-bit words as hex (64 per line), then `END sum=..` |
| `pull <none\|up\|down\|both>` | `PULL mode=..` (internal ~45 kΩ pulls on the sampled pin; `both` = mid-rail) |
| `src <32\|33>` | `SRC gpio=.. ch=..` (switch to the GPIO33 control pin and back) |
| `disp <on\|off>` | `DISP on=..` (sleeps the ST7789 + backlight to rule it out as a noise source) |

Raw word layout: bits 15–12 hold the ADC channel, bits 11–0 the 12-bit sample. The I2S FIFO swaps each pair of 16-bit samples, so un-swap them (`buf[i ^ 1]`) before use.

## Limits of the measurement

- **Input range:** 12 dB attenuation covers about 0.14–3.15 V. Anything above reads 4095. A 40 kHz signal must swing inside that range, ideally centred around 1.6 V.
- **Rail ripple:** board and USB noise produces a faint ~41 kHz / 83 kHz line, around 0.3 mV, even on an unconnected pin. Treat a tone that weak as interference, not signal.
- **Accuracy:** the ESP32 ADC is not an oscilloscope (~10 mV RMS noise at 249 kS/s). Use a scope to look at waveform shape.
