# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Workspace for ESP32 projects targeting a **LilyGO T-Display v1.1** (ESP32-D0WDQ6-V3, 4MB flash, no PSRAM, ST7789 135x240 TFT), built and flashed from WSL2 with PlatformIO. Each subfolder is a standalone PlatformIO project. `tdisplay_test/` is the working reference: copy its `platformio.ini` when starting a new project. See `README.md` for the user-facing guide.

## Toolchain versions (matter for API choice)

- PlatformIO Core 6.2 (`~/.local/bin/pio`, installed with `uv tool install --python 3.13 platformio`).
- Platform `espressif32` 7.1.3, which ships **Arduino-ESP32 core 2.0.17** (ESP-IDF 4.4). Use the 2.x APIs: `ledcSetup()`/`ledcAttachPin()`, not the 3.x `ledcAttach()`; `analogReadMilliVolts()` is available.
- `bodmer/TFT_eSPI@^2.5.43`.

## Commands

Run from inside a project folder (e.g. `~/esp32/tdisplay_test`):

```bash
pio run                      # build
pio run -t upload            # build + flash (port pinned to /dev/ttyACM0 in platformio.ini)
pio run -t clean
pio pkg exec -p tool-esptoolpy -- esptool.py --port /dev/ttyACM0 flash_id   # check serial link, chip, flash size
```

There are no tests or linters configured.

`pio device monitor` needs a TTY, so Claude cannot use it. Read serial without it like this (it resets the board to capture the boot output):

```bash
~/.local/share/uv/tools/platformio/bin/python - <<'EOF'
import serial, time
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.2
s.dtr = False; s.rts = False; s.open()
s.rts = True; time.sleep(0.1); s.rts = False   # pulse EN = reset
end = time.time() + 8; buf = b""
while time.time() < end: buf += s.read(4096)
print(buf.decode(errors="replace"))
EOF
```

## USB connection (WSL2 + usbipd-win)

The board reaches WSL through usbipd-win (BUSID `6-1`, VID:PID `1a86:55d4`, CH9102 bridge, appears as `/dev/ttyACM0` via `cdc_acm`). Before uploading, check `ls /dev/ttyACM0`. If it is missing, (re)attach it from WSL, running in the background:

```bash
"/mnt/c/Program Files/usbipd-win/usbipd.exe" attach --wsl --busid 6-1 --auto-attach
```

The device is already bound (persisted), so `bind` (admin) is not needed again. If the BUSID changed (different USB port), find it with `usbipd.exe list`. Port permissions come from PlatformIO's udev rules (`/etc/udev/rules.d/99-platformio-udev.rules`, mode 0666) and from `dialout` membership.

`sudo` needs a password, so ask the user to run sudo commands themselves with `! <command>`. A pre-existing half-configured `openssh-server` package makes every `apt-get` call exit 1 even when the install worked, so never chain anything after `apt-get` with `&&`.

## Display configuration (non-obvious)

TFT_eSPI is configured **entirely through `build_flags`** in `platformio.ini`: `-DUSER_SETUP_LOADED=1` plus the pins from TFT_eSPI's `Setup25_TTGO_T_Display.h`. Never edit `User_Setup.h`/`User_Setup_Select.h` under `.pio/libdeps/`, because PlatformIO re-downloads that folder. Key flags:
- `CGRAM_OFFSET=1` is required: the 135x240 panel sits at an offset inside the controller's 240x320 RAM. Without it, the image is shifted and shows garbage borders.
- Fonts are compiled in only when their `LOAD_*` flag is set (GLCD, 2, 4, 6, 7, 8, GFXFF, SMOOTH_FONT are all enabled).
- `setRotation(1)` gives 240x135 landscape.

## Board pin facts

| Function | GPIO | Note |
|---|---|---|
| TFT MOSI/SCLK/CS/DC/RST/BL | 19/18/5/16/23/4 | backlight active HIGH |
| Button left (BOOT) | 0 | `INPUT_PULLUP`, LOW = pressed; held at reset = download mode |
| Button right | 35 | input-only, no internal pull-up (external one on board), LOW = pressed |
| Supply voltage ADC | 34 | ×2 divider; set GPIO14 (ADC_EN) HIGH first |

GPIOs 34–39 are input-only. Keep project folders on the WSL ext4 filesystem (`~/esp32`), not `/mnt/c`, because builds there are much slower.
