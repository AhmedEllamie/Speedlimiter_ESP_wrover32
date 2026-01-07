# Speed Limiter System - PlatformIO Configuration

## Project Information
- **Platform**: ESP32-WROVER-E-N4R8
- **Framework**: Arduino + ESP-IDF drivers
- **Firmware**: `src/main.cpp`
- **Build System**: PlatformIO
- **Date**: January 5th, 2026

## Quick Start

### Prerequisites
1. Install PlatformIO extension in VS Code
2. Install Python 3.6+ and pip
3. Install PlatformIO CLI: `pip install platformio`

### Build Instructions

#### Default Build (PWM Mode - Recommended)
```bash
# Build for ESP32-WROVER-E-N4R8 with PWM output (IO21/IO22)
pio run

# Upload to board
pio run --target upload

# Upload and monitor serial
pio run --target upload --target monitor

# Clean build
pio run --target clean
```

#### DAC Mode Build (Optional)
```bash
# Build with DAC output on IO25/IO26
pio run -e esp32-wrover-n4r8-dac

# Upload to board
pio run -e esp32-wrover-n4r8-dac --target upload

# Upload and monitor
pio run -e esp32-wrover-n4r8-dac --target upload --target monitor
```

#### Debug Build
```bash
# Build with debug symbols
pio run -e esp32-wrover-n4r8-pwm-debug

# Start debug session
pio debug -e esp32-wrover-n4r8-pwm-debug
```

## Configuration Options

### Environments

| Environment | Description | Output Method | Pins | Default |
|-------------|---------------|----------------|-------|-----------|
| `esp32-wrover-n4r8-pwm` | PWM mode (recommended) | PWM on IO21/IO22 | ✅ YES |
| `esp32-wrover-n4r8-dac` | DAC mode (true analog) | DAC on IO25/IO26 | NO |
| `esp32-wrover-n4r8-pwm-debug` | PWM with debug | PWM on IO21/IO22 | NO |

### Choosing Output Method

#### PWM Mode (Recommended)
- **Build**: `pio run` (uses default environment)
- **Output Pins**: IO21 (SDPS1), IO22 (SDPS2)
- **Resolution**: 10-bit (0-1023, ~3.2 mV/step)
- **Hardware**: Requires RC filter (1kΩ + 10µF)
- **Pros**:
  - Matches SRD pin assignment
  - Higher resolution (10-bit vs 8-bit DAC)
  - Proven reliability
  - Automotive standard
- **Cons**:
  - Requires external filter components
  - Switching noise (filtered out)

#### DAC Mode (Optional)
- **Build**: `pio run -e esp32-wrover-n4r8-dac`
- **Output Pins**: IO25 (SDPS1), IO26 (SDPS2)
- **Resolution**: 8-bit (0-255, ~12.9 mV/step)
- **Hardware**: No filter needed (true analog)
- **Pros**:
  - True analog output
  - No external components
  - Simple design
  - Low EMI/EMC noise
- **Cons**:
  - Lower resolution (8-bit vs 10-bit)
  - Doesn't match SRD pins
  - Pin conflicts with potentiometers
  - Not tested with old firmware

### Changing Output Method at Runtime

To change between PWM and DAC:

1. **Edit `platformio.ini`**:
   ```ini
   [platformio]
   default_envs = esp32-wrover-n4r8-pwm    ; For PWM
   # default_envs = esp32-wrover-n4r8-dac  ; For DAC
   ```

2. **Rebuild firmware**:
   ```bash
   pio run --target clean
   pio run
   pio run --target upload
   ```

3. **Update hardware**:
   - **PWM mode**: Add RC filters (1kΩ + 10µF) to IO21/IO22
   - **DAC mode**: Direct connection to IO25/IO26 (no filter)

## Hardware Requirements

### For PWM Mode
```
┌─────────────────────────────────────────────┐
│              ESP32 Board               │
│  ┌───────────────────────────────┐    │
│  │  IO21  ──[1kΩ]─┬───►    │    │
│  │  (PWM)            │          │    │
│  │                     ─┴─10µF   │    │
│  │                      ─┬─       │    │
│  │  IO22  ──[1kΩ]─┬───►    │    │
│  │  (PWM)            │          │    │
│  │                     ─┴─10µF   │    │
│  │                      ─┬─       │    │
│  └───────────────────────┴─────────┘    │
│                                        │
└─────────────────────────────────────────────┘
```

**Components per output:**
- 1x Resistor: 1kΩ, 1/4W, 1% tolerance
- 1x Capacitor: 10µF, 25V, ceramic or tantalum
- Filter cutoff: ~16 Hz (well below 5 kHz PWM frequency)

### For DAC Mode
```
┌─────────────────────────────────────────────┐
│              ESP32 Board               │
│  ┌───────────────────────────────┐    │
│  │  IO25  ────────────►          │    │
│  │  (DAC)                      │    │
│  │                               │    │
│  │  IO26  ────────────►          │    │
│  │  (DAC)                      │    │
│  └───────────────────────────────┘    │
│                                        │
└─────────────────────────────────────────────┘
```

**Components**: None (direct connection)

## Library Dependencies

PlatformIO automatically installs these libraries:

- **esp32-can**: CAN bus driver
- **Preferences**: Non-volatile storage (built into ESP32 Arduino core)

## Serial Monitor

### USB Serial (Debug/Diagnostics)
- **Port**: USB
- **Baud**: 115200
- **Usage**: `pio device monitor` or VS Code Serial Monitor

### RS232 Serial (Configuration/Diagnostics)
- **TX Pin**: IO15
- **RX Pin**: IO34
- **Baud**: 115200 (configurable via CLI)
- **Usage**: External RS232 interface

## Build System

### Directory Structure
```
SPEED_LIMITER/
├── src/main.cpp                       # Main firmware
├── platformio.ini                      # PlatformIO configuration
├── README_PlatformIO.md               # This file
├── Software_Requirements_Document.txt  # Requirements (SRD)
├── SRD_Comparison_Differences.txt      # Notes/differences vs older firmware
├── NewFirmware_Test_Procedure_Detailed.txt # Test procedure
└── .pio/                              # Build output (generated)
    ├── build/
    └── esp32-wrover-n4r8/
```

### Build Output
After building:
- Firmware: `.pio/build/esp32-wrover-n4r8-pwm/firmware.bin`
- Partition table: `partitions.bin`
- Bootloader: `bootloader.bin`

### Upload
PlatformIO automatically:
1. Compiles firmware
2. Generates binaries
3. Uploads via USB/UART
4. Resets ESP32

## Troubleshooting

### Build Issues

**Error: "Unknown board"**
```bash
# Install ESP32 platform
pio platform install espressif32

# List available boards
pio boards espressif32
```

**Error: "Library not found"**
```bash
# Install library manually
pio lib install esp32-can
pio lib install Preferences
```

**Error: "Compilation failed"**
- Check Arduino/ESP-IDF version: `pio platform update`
- Verify dependencies in `platformio.ini`
- Check for typos in `src/main.cpp`

### Upload Issues

**Error: "Failed to connect"**
```bash
# Check serial port
pio device list

# Manually specify port
pio run --target upload --upload-port COM3
```

**Error: "Write timeout"**
- Disconnect all other serial monitors
- Press BOOT button while uploading
- Check USB cable (use data cable, not charge-only)

### Runtime Issues

**Error: "ADC init failed"**
- Check wiring to IO39/IO36/IO25/IO26
- Verify ADC unit configuration
- Check for short circuits

**Error: "CAN init failed"**
- Check CAN bus wiring (IO32/IO33)
- Verify CAN baud rate (250k or 500k)
- Check CAN termination resistor (120Ω)

**Error: "Calibration rejected"**
- Ensure pedal is at rest position
- Check for loose connections
- Verify ADC readings are stable

## Advanced Configuration

### Custom Build Flags
To add custom defines:
```ini
[env:esp32-wrover-n4r8-pwm-custom]
extends = env:esp32-wrover-n4r8-pwm
build_flags =
    ${env.build_flags}
    -DCUSTOM_DEFINE=value
```

### Multiple Firmware Variants
To build both PWM and DAC in one command:
```bash
# Build all environments
pio run

# Or specific list
pio run -e esp32-wrover-n4r8-pwm esp32-wrover-n4r8-dac
```

### Debugging
To debug with ESP-PROG or J-Link:
1. Connect debug probe to ESP32 JTAG pins
2. Build debug firmware: `pio run -e esp32-wrover-n4r8-pwm-debug`
3. Start debug: `pio debug -e esp32-wrover-n4r8-pwm-debug`

## Testing

### Unit Tests
```bash
# Run tests (if implemented)
pio test
```

### Memory Usage
```bash
# Check memory usage
pio run --target size
```

Expected output:
```
Sketch uses 123456 bytes (37%) of program storage space. Maximum is 327680 bytes.
Global variables use 1234 bytes (6%) of dynamic memory.
```

## CI/CD Integration

### GitHub Actions Example
```yaml
name: Build
on: [push]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: marvinpinto/action-platformio@v0.2.0
        with:
          platformio_ini: platformio.ini
          args: run
```

## Support

### Documentation
- **Firmware**: See `src/main.cpp` comments
- **SRD**: `Software_Requirements_Document.txt`
- **Old vs New**: `SRD_Comparison_Differences.txt`
- **Test Procedure**: `NewFirmware_Test_Procedure_Detailed.txt`

### CLI Commands
Connect via USB Serial (115200 baud) and type:
- `help` - Show all commands
- `status` - Show system status
- `calibrate` - Calibrate Mode 2 defaults
- `set limit <kmh>` - Set speed limit
- `set factor <float>` - Set speed calibration factor
- `set offset <float>` - Set speed calibration offset
- `factory_reset` - Clear all configuration

## Version History

### v1.0 (2025-01-05)
- Initial PlatformIO configuration
- PWM mode (default)
- DAC mode (optional)
- Debug configuration
- Support for ESP32-WROVER-E-N4R8

## License

This firmware is part of the Speed Limiter System project.
See project documentation for licensing information.

---

**Happy Building! 🚀**

For issues or questions, refer to:
1. Project documentation files
2. ESP32 PlatformIO documentation: https://docs.platformio.org/
3. ESP32 Arduino documentation: https://docs.espressif.com/projects/arduino-esp32

