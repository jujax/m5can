# M5Stack Core2 CAN Monitor

A professional CAN bus monitor and transmitter for M5Stack Core2 with COMMU module. Send and receive automotive CAN frames at 500 kbps with a beautiful touchscreen interface.

![M5Stack Core2 CAN Monitor](https://img.shields.io/badge/M5Stack-Core2-blue)
![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-green)
![CAN Speed](https://img.shields.io/badge/CAN-500kbps-orange)

## Features

- ✅ **CAN Bus Communication** at 500 kbps
- ✅ **OBD-II Frame Library** - Pre-configured automotive frames (RPM, Speed, Temperature, etc.)
- ✅ **Real-time Display** - Beautiful touchscreen interface with battery monitoring
- ✅ **Frame Transmission** - Send CAN frames automatically or manually
- ✅ **Frame Reception** - Log and display received CAN messages
- ✅ **Battery Monitoring** - Real-time battery level and charging status
- ✅ **Touch Controls** - Navigate frames with touch gestures
- ✅ **Serial Debug** - Optional serial output for debugging
- ✅ **Power Management** - Automatic screen dimming and fast charging support
- ✅ **Battery Saving** - Screen auto-off after inactivity, optimized power consumption
- ✅ **SD Card Logging** - Record all CAN frames (TX and RX) to SD card in CSV format

## Hardware Requirements

- **M5Stack Core2** - Main controller
- **Module COMMU** - CAN/RS485/TTL communication module
- **CAN Bus** - 500 kbps (configurable)

### Pin Mapping (M5-Bus)

| Signal | GPIO | M5-Bus Position |
|--------|------|-----------------|
| CS     | 27   | 21              |
| INT    | 2    | 23              |
| MOSI   | 23   | 7               |
| MISO   | 38   | 9               |
| SCK    | 18   | 11              |

**Important**: Remove the base from Core2 when using the COMMU module (as per M5Stack documentation).

## Installation

### Prerequisites

- [PlatformIO](https://platformio.org/) installed
- USB Type-C cable

### Setup

1. Clone this repository:
```bash
git clone https://github.com/jujax/m5can.git
cd m5can
```

2. Build and upload:
```bash
pio run -t upload
```

3. Monitor serial output:
```bash
pio device monitor
```

## Usage

### Controls

- **Button A**: Play/Pause automatic frame transmission (first press wakes screen if off)
- **Button B**: Next frame (cycle through available frames)
- **Button C (short)**: Toggle serial debug output
- **Button C (long 1s)**: Cycle brightness (100% → 30% → 10%)
- **Button C (long 2s)**: Toggle SD card logging
- **Long Press Left Screen**: Previous frame
- **Long Press Right Screen**: Reset TX/RX counters
- **Long Press Header**: Cycle brightness levels
- **Long Press SD Icon**: Toggle SD card logging
- **Double Tap Header**: Toggle screen on/off instantly

### Power Management

The device includes smart power management features:

| Feature | Description |
|---------|-------------|
| **Fast Charging** | Up to 780mA charge current for faster battery charging |
| **Auto Dim** | Screen dims to 30% after 15 seconds of inactivity |
| **Auto Off** | Screen turns off after 60 seconds of inactivity |
| **Manual Brightness** | Long press Button C or header to cycle brightness |
| **Instant Wake** | Any button or touch immediately wakes the screen |
| **Power Saving** | Unused peripherals (vibration motor) disabled |

**Note**: CAN communication continues even when the screen is off!

### Available CAN Frames

The project includes pre-configured OBD-II and automotive frames:

- **RPM** - Engine RPM (0x7DF)
- **Speed** - Vehicle speed (0x7DF)
- **Coolant** - Coolant temperature (0x7DF)
- **Intake** - Intake air temperature (0x7DF)
- **Throttle** - Throttle position (0x7DF)
- **Fuel** - Fuel level (0x7DF)
- **Load** - Engine load (0x7DF)
- **Voltage** - Battery voltage (0x7DF)
- **VIN** - Vehicle identification number (0x7DF)
- **RPM BMW** - BMW E46 RPM frame (0x316)
- **Speed VAG** - VAG speed frame (0x153)
- **RPM PSA** - PSA RPM frame (0x280)

### Display Interface

The screen shows:
- **Header**: CAN status LED, brightness indicator, SD card status, charging indicator, battery level
- **TX Section**: Current frame being sent (name, ID, data)
- **RX Log**: Last 4 received CAN messages
- **Counters**: Total TX and RX frame counts

### SD Card Logging

All CAN frames (both TX and RX) can be logged to the SD card in CSV format:

**File Format:**
```
=== M5Stack CAN Bus Logger ===
Started: 12345 ms
Format: timestamp_ms,type,id,length,data_hex
---
12345,TX,0x7DF,8,02 01 0C 55 55 55 55 55
12350,RX,0x7E8,8,04 41 0C 1A F0 00 00 00
```

**Features:**
- Automatic file rotation when file size exceeds 10MB
- Files named `can_log_0.txt`, `can_log_1.txt`, etc.
- Data flushed every 5 seconds to prevent data loss
- Visual indicator in header (green = logging active, red dot = recording)

**Requirements:**
- SD card must be inserted before power-on
- SD card will be initialized automatically on startup

## Configuration

### CAN Speed

Default is 500 kbps. To change, edit `src/main.cpp`:

```cpp
#define CAN_SPEED    CAN_500KBPS  // Change to CAN_250KBPS, CAN_125KBPS, etc.
```

### Adding Custom Frames

Edit the `carFrames` array in `src/main.cpp`:

```cpp
const CarFrame carFrames[] = {
    {0x7DF, 8, {0x02, 0x01, 0x0C, 0x55, 0x55, 0x55, 0x55, 0x55}, "RPM"},
    // Add your custom frames here
    {0x123, 8, {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11}, "Custom"},
};
```

### Power Management Settings

Adjust power saving timeouts in `src/main.cpp`:

```cpp
#define BRIGHTNESS_MAX       100    // Max brightness (%)
#define BRIGHTNESS_DIM       30     // Dimmed brightness (%)
#define BRIGHTNESS_MIN       10     // Minimum before off (%)
#define TIMEOUT_DIM_MS       15000  // 15s before dimming
#define TIMEOUT_OFF_MS       60000  // 60s before screen off
#define CHARGE_CURRENT_FAST  0x0F   // 780mA charge (see values below)
```

**Charge Current Values (AXP192):**
| Value | Current | Value | Current |
|-------|---------|-------|---------|
| 0x00  | 100mA   | 0x08  | 780mA   |
| 0x02  | 280mA   | 0x0A  | 960mA   |
| 0x04  | 450mA   | 0x0C  | 1080mA  |
| 0x06  | 630mA   | 0x0F  | 1320mA  |

⚠️ **Warning**: Higher charge currents generate more heat. 780mA is recommended for safe fast charging.

## Project Structure

```
m5can/
├── src/
│   └── main.cpp          # Main application code
├── platformio.ini        # PlatformIO configuration
└── README.md            # This file
```

## Dependencies

- **M5Core2** (^0.1.9) - M5Stack Core2 library
- **mcp_can** (^1.5.1) - MCP2515 CAN controller library

## Troubleshooting

### CAN Not Initializing

- Verify COMMU module is properly connected
- Ensure base is removed from Core2
- Check SPI connections (CS, MOSI, MISO, SCK)
- Verify CAN bus termination (120Ω resistors)

### No Frames Received

- Check CAN bus wiring
- Verify bus speed matches other devices
- Ensure proper CAN termination
- Check if other devices are transmitting

### Display Issues

- Reboot the device
- Check if touchscreen is responding
- Verify M5Core2 library version

### SD Card Issues

- Ensure SD card is inserted before power-on
- Use SDHC cards (up to 32GB) for best compatibility
- Format SD card as FAT32 if issues occur
- Check that SD card is not write-protected
- Verify SD card is properly seated in the slot
- If logging fails, check serial output for error messages

## License

This project is open source and available under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- [M5Stack](https://m5stack.com/) for the excellent hardware
- [PlatformIO](https://platformio.org/) for the development platform
- MCP2515 CAN controller library by Cory J. Fowler

## Related Links

- [M5Stack Core2 Documentation](https://docs.m5stack.com/en/core/core2)
- [Module COMMU Documentation](https://docs.m5stack.com/en/module/commu)
- [MCP2515 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-Data-Sheet-20001801J.pdf)

---

