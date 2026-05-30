# SH1106 OLED Display Driver for PX4

A PX4 driver module for the SH1106 128×64 OLED via I²C.  
Displays flight mode, arming state, battery status, GPS, and link info.

---

## Features
- Works with PX4 FMU v6C (tested)
- Lightweight, non-blocking task
- 6×8 bitmap font rendering
- Shows mode, battery, GPS, and RSSI

---

## Build & Install
1. Place this folder under `src/drivers/oled_display/` in PX4.
2. Ensure `CMakeLists.txt` and `module.yaml` are included in your board config.
3. Build as usual: make px4_fmu-v6c_default

## Usage (NSH / MAVLink Console)

- Start the driver (bus 2, address 0x3C):  
  `sh1106_notify start -b 2 -a 0x3c`

- Other commands:  
  - `sh1106_notify status`  
  - `sh1106_notify stop`

- Example screen:  
  - `POSCTL | DISARM`  
  - `Batt: 11.98V 73%`  
  - `GPS: 3D sats:12`  
  - `Link: RSSI:81 tx%:92 err:0`



## Repository Layout

- CMakeLists.txt
- Kconfig
- module.yaml
- font6x8.cpp / font6x8.hpp
- sh1106.cpp / sh1106.hpp
- sh1106_notify.cpp
