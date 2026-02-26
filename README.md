# esp-mouse-jigger

BLE mouse jiggler using ESP32-C6. Simulates human-like cursor movements and scrolling over Bluetooth HID to keep your computer awake.

## Features

- Appears as a standard Bluetooth mouse (HID over GATT)
- Human-like cursor movements with ease-in-out acceleration
- Random scrolling and micro-adjustments
- Subtle hand jitter for realistic behavior
- Auto-reconnect after reboot (bonding data persisted in NVS)
- No driver or software needed on the host

## Requirements

- ESP32-C6 development board
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) v5.5+

## Build & Flash

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/cu.usbmodem2101 flash
```

Replace `/dev/cu.usbmodem2101` with your serial port.

## Usage

1. Power the ESP32-C6 via USB
2. Open Bluetooth settings on your computer
3. Find **ESP32-C6 Mouse** and connect
4. The cursor will start moving on its own

After the initial pairing, the device reconnects automatically on reboot.

## How It Works

The firmware registers a BLE HID service with a standard mouse report descriptor (3 buttons + X/Y/wheel). A FreeRTOS task continuously generates randomized mouse reports:

- **Smooth moves** — cursor travels to random points with ease-in-out easing
- **Micro-adjustments** — small corrections simulating hovering
- **Scrolling** — random up/down scroll bursts

Pairing uses "Just Works" (no PIN required). Bond keys are stored in NVS flash so the host can reconnect without re-pairing.

## Project Structure

```
├── CMakeLists.txt
├── sdkconfig.defaults
└── main/
    ├── CMakeLists.txt
    └── main.c
```

## License

MIT
