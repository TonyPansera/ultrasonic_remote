# ESP32 LilyGO T-Display: development from WSL2

This folder holds ESP32 projects for the **LilyGO T-Display v1.1**. They are built and flashed from WSL2 (Ubuntu) with **PlatformIO**. Each subfolder is an independent project.

| Project | What it does |
|---|---|
| [`tdisplay_test/`](tdisplay_test) | Hardware test (screen, buttons, chip info) and template for new projects |
| [`ultrasonic_probe/`](ultrasonic_probe) | Samples GPIO32 at ~249 kS/s and checks for a 40 kHz ultrasonic signal; host script makes graphs and a report ([latest report](ultrasonic_probe/results/REPORT.md)) |

Windows path to this folder (for VS Code or Explorer): `\\wsl.localhost\Ubuntu-26.04\home\tonyp\esp32`

---

## 1. The board

| Item | Value |
|---|---|
| Chip | ESP32-D0WDQ6-V3 (rev 3.1), dual core 240 MHz, Wi-Fi + Bluetooth |
| Memory | 4 MB flash, 520 KB RAM, no PSRAM |
| Screen | 1.14" ST7789 IPS, 135 × 240 px, SPI |
| USB-serial chip | WCH CH9102 (`1a86:55d4`), seen as `COM3` on Windows and `/dev/ttyACM0` in WSL |
| Buttons | left = GPIO0 (also BOOT), right = GPIO35, side = RST (reset) |

### Pinout used by the board itself

| Function | GPIO |
|---|---|
| TFT MOSI | 19 |
| TFT SCLK | 18 |
| TFT CS | 5 |
| TFT DC | 16 |
| TFT RST | 23 |
| TFT backlight | 4 (HIGH = on) |
| Button left | 0 (LOW when pressed) |
| Button right | 35 (LOW when pressed) |
| Battery/USB voltage | 34 (analog, ×2 divider) |
| Voltage divider enable | 14 (set HIGH before reading GPIO34) |

GPIO 34, 35, 36 and 39 are **input only**: they cannot drive LEDs or anything else, and they have no internal pull-up resistors.

---

## 2. What is installed (one-time setup, already done)

You only need this section to rebuild the setup on another PC.

1. **usbipd-win** (Windows) forwards the USB device into WSL2.
   ```bash
   winget.exe install --exact --id dorssel.usbipd-win
   ```
2. **Share the board once** (needs admin, so a UAC prompt appears). Find the BUSID first with `usbipd.exe list`.
   ```bash
   powershell.exe -Command "Start-Process -FilePath 'C:\Program Files\usbipd-win\usbipd.exe' -ArgumentList 'bind','--busid','3-2' -Verb RunAs -Wait"
   ```
3. **Serial port permissions** (WSL):
   ```bash
   curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
     | sudo tee /etc/udev/rules.d/99-platformio-udev.rules >/dev/null
   sudo udevadm control --reload-rules && sudo udevadm trigger
   sudo usermod -aG dialout $USER
   sudo apt-get install -y usbutils     # provides lsusb
   ```
4. **PlatformIO**, installed with `uv` on Python 3.13:
   ```bash
   uv tool install --python 3.13 platformio
   ```
   This provides `pio` in `~/.local/bin`. The ESP32 compiler, Arduino framework and `esptool` are downloaded automatically (about 1.5 GB in `~/.platformio`) the first time a project is built.

Installed versions: PlatformIO 6.2, platform `espressif32` 7.1.3, **Arduino-ESP32 core 2.0.17**, TFT_eSPI 2.5.43.

---

## 3. Connecting the board (every session)

USB devices are not shared with WSL automatically. After each Windows reboot or board replug, attach the board again:

```bash
"/mnt/c/Program Files/usbipd-win/usbipd.exe" attach --wsl --busid 3-2 --auto-attach
```

- `--auto-attach` keeps the command running and re-attaches the board whenever it is unplugged and plugged back in. Leave that terminal open, or press Ctrl+C to stop.
- No admin rights are needed for `attach`. The admin-only `bind` step was done once and is remembered.
- The BUSID depends on the **physical USB port**: `3-2` on the current port, `6-1` on the other one. After switching ports, run `"/mnt/c/Program Files/usbipd-win/usbipd.exe" list` and look for `1a86:55d4 USB-Enhanced-SERIAL CH9102`. The share (`bind`) follows the device, so only `attach` needs the new BUSID. Stop any old `--auto-attach` loop first.

Optional shortcut, to add to `~/.bashrc`:

```bash
alias usbipd='"/mnt/c/Program Files/usbipd-win/usbipd.exe"'
alias esp-attach='usbipd attach --wsl --busid 3-2 --auto-attach'
```

**Check that it is connected:**

```bash
lsusb | grep 1a86        # Bus 001 Device 002: ID 1a86:55d4 QinHeng Electronics USB Single Serial
ls -l /dev/ttyACM0       # crw-rw-rw- ... /dev/ttyACM0
```

**Giving the board back to Windows** (for Arduino IDE on Windows, for example): while the board is attached to WSL, `COM3` does not exist on Windows. Detach it with:

```bash
"/mnt/c/Program Files/usbipd-win/usbipd.exe" detach --busid 3-2
```

---

## 4. Daily workflow

Run everything from **inside a project folder**:

```bash
cd ~/esp32/tdisplay_test
```

| Action | Command |
|---|---|
| Compile only | `pio run` |
| Compile + flash | `pio run -t upload` |
| Open serial monitor | `pio device monitor` (quit with **Ctrl+C**) |
| Flash, then open monitor | `pio run -t upload -t monitor` |
| Delete build files | `pio run -t clean` |
| Check chip / flash / connection | `pio pkg exec -p tool-esptoolpy -- esptool.py --port /dev/ttyACM0 flash_id` |

Notes:
- The first build of a new project takes a few minutes because the Arduino framework is compiled. Later builds only recompile what changed (a few seconds).
- The serial monitor and the upload both use `/dev/ttyACM0`. **Close the monitor before uploading**, otherwise the upload fails with "port is busy". `pio run -t upload -t monitor` handles this for you.
- The upload resets the board automatically. No button press is needed.
- `Serial.begin(115200)` in the code must match `monitor_speed = 115200` in `platformio.ini`.

---

## 5. Creating a new project

The simplest way is to copy the test project, which already contains the working display configuration:

```bash
cd ~/esp32
cp -r tdisplay_test my_project
rm -rf my_project/.pio          # don't copy old build files
cd my_project
# edit src/main.cpp, then:
pio run -t upload -t monitor
```

Project layout:

```
my_project/
├── platformio.ini     # board, libraries, display pins, serial speeds
├── src/main.cpp       # your code (Arduino style: setup() + loop())
├── include/           # optional: your own .h files
├── lib/               # optional: your own local libraries
└── .pio/              # generated: build output + downloaded libraries (never edit)
```

Keep projects in `~/esp32` (Linux filesystem). Building from `/mnt/c/...` works too but is much slower.

### Adding a library

Add it to `lib_deps` in `platformio.ini`, one per line:

```ini
lib_deps =
  bodmer/TFT_eSPI@^2.5.43
  adafruit/DHT sensor library@^1.4.6
```

Search for libraries with `pio pkg search "dht"`. The next `pio run` downloads them automatically.

---

## 6. Using the screen (TFT_eSPI)

The display configuration is **in `platformio.ini` (`build_flags`)**, not in the library. Never edit `User_Setup.h` inside `.pio/libdeps/`, because PlatformIO overwrites that folder. If you start a project without copying `tdisplay_test`, copy its whole `build_flags` block.

Minimal example:

```cpp
#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft;

void setup() {
  tft.init();
  tft.setRotation(1);                     // 0/2 = portrait 135x240, 1/3 = landscape 240x135
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // text color, background color
  tft.drawString("Hello!", 10, 10, 4);    // text, x, y, font number
}

void loop() {}
```

Most useful functions:

| Function | Use |
|---|---|
| `fillScreen(color)` | clear the screen |
| `drawString(text, x, y, font)` | draw text; position set by `setTextDatum()` (`TL_DATUM` top-left, `MC_DATUM` centered…) |
| `setTextColor(fg, bg)` | giving a background color makes text overwrite the old text cleanly |
| `setTextPadding(width)` | clears a fixed width behind the text, for values that change length (counters…) |
| `drawPixel`, `drawLine`, `drawRect`, `fillRect`, `fillRoundRect`, `drawCircle`, `fillCircle` | shapes |
| `width()`, `height()` | screen size for the current rotation |
| `color565(r, g, b)` | build a custom color |

Built-in fonts: `1` (8 px), `2` (16 px), `4` (26 px), `6` (48 px, digits only), `7` (48 px 7-segment, digits only), `8` (75 px, digits only). Named colors: `TFT_BLACK`, `TFT_WHITE`, `TFT_RED`, `TFT_GREEN`, `TFT_BLUE`, `TFT_YELLOW`, `TFT_CYAN`, `TFT_ORANGE`, `TFT_DARKGREY`…

To avoid flicker, redraw only what changed. Don't call `fillScreen()` in `loop()`. For complex animations, look at `TFT_eSprite` (draw off-screen, then push the whole image at once).

Library examples are in `.pio/libdeps/tdisplay/TFT_eSPI/examples/`, once a project has been built.

### Buttons and voltage

```cpp
pinMode(0, INPUT_PULLUP);        // left button
pinMode(35, INPUT);              // right button (has an external pull-up)
bool leftPressed  = digitalRead(0)  == LOW;
bool rightPressed = digitalRead(35) == LOW;

pinMode(14, OUTPUT); digitalWrite(14, HIGH);          // enable the voltage divider
float volts = analogReadMilliVolts(34) * 2 / 1000.0;  // ≈4.7–5.0 V on USB, 3.3–4.2 V on battery
```

---

## 7. Arduino-ESP32 version note

The installed framework is **Arduino-ESP32 2.0.17**. Many tutorials online are written for version 3.x, whose API is different in a few places. The most common one is PWM:

```cpp
// 2.0.x (what we have)
ledcSetup(0, 5000, 8);   // channel, frequency, resolution bits
ledcAttachPin(25, 0);    // pin, channel
ledcWrite(0, 128);       // channel, duty

// 3.x (will NOT compile here)
// ledcAttach(25, 5000, 8); ledcWrite(25, 128);
```

If example code fails with "was not declared in this scope", check whether it targets core 3.x.

---

## 8. The test program (`tdisplay_test`)

What it does:
1. At boot, fills the screen red, green, blue, then white (0.5 s each), to check colors and dead pixels.
2. Shows "Hello T-Display!", the chip model, flash size and MAC address.
3. Updates the uptime and supply voltage every second.
4. Shows two boxes at the bottom that turn **green** while the GPIO0 / GPIO35 buttons are held.
5. Prints the same information on the serial port at 115200 baud:
   ```
   === T-Display test ===
   Chip: ESP32-D0WDQ6-V3 rev3, 2 cores, 240 MHz
   Flash: 4 MB, PSRAM: 0 B, heap free: 349308 B
   MAC: A0:DD:6C:72:CD:2C
   Display: 240x135, rotation 1
   uptime=2s supply=4.74V btn0=0 btn35=0
   ```

Flash it again any time to check that the board and toolchain still work.

---

## 9. Troubleshooting

| Problem | Fix |
|---|---|
| `/dev/ttyACM0` does not exist | Board not attached to WSL: run the `usbipd attach` command (section 3). Check the cable: some USB-C cables are charge-only. |
| `usbipd: error: There is no device with busid '3-2'` | Board on another USB port, or not plugged in. Run `usbipd.exe list` to get the new BUSID. If the device shows as "Not shared", repeat the `bind` step (admin). |
| `Permission denied: '/dev/ttyACM0'` | udev rules missing (section 2, step 3). Check with `ls -l /dev/ttyACM0`, which should show `crw-rw-rw-`. |
| `could not open port ... Device or resource busy` | A serial monitor is still open (here or on Windows). Close it. |
| `Failed to connect to ESP32: Wrong boot mode detected` / `Timed out waiting for packet header` | Enter download mode by hand: **hold the left button (GPIO0/BOOT), press and release RST, release GPIO0**, then upload again. |
| Upload stops halfway with errors | Lower the speed in `platformio.ini`: `upload_speed = 460800`. |
| Screen stays black | Backlight not on: check that `-DTFT_BL=4` and `-DTFT_BACKLIGHT_ON=HIGH` are in `build_flags`, and that `tft.init()` is called. |
| Image shifted, random pixels at the edges | `-DCGRAM_OFFSET=1` is missing from `build_flags`. |
| Text in a font does not appear | That font's `-DLOAD_FONTx=1` flag is missing from `build_flags`. |
| Garbage characters in the serial monitor | Baud mismatch: `monitor_speed` must equal the `Serial.begin()` value. |
| Board crashes in a loop (`Guru Meditation Error`) | Read the error in the monitor. Common causes: an array index out of range, a null pointer, or a stack overflow from big local arrays (make them `static` or global). |
| `COM3` missing on Windows | The board is attached to WSL. Use `usbipd detach --busid 3-2` (section 3). |
| `apt-get` ends with an `openssh-server` error | Pre-existing broken package, unrelated to the ESP32. The rest of the install still succeeded. |
