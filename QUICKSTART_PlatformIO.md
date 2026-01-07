# Quick Start Guide - PlatformIO

## What is this?
PlatformIO configuration files to build your Speed Limiter firmware using PlatformIO instead of Arduino IDE.

## Files Created

1. **`platformio.ini`** - PlatformIO configuration (main file)
2. **`README_PlatformIO.md`** - Complete documentation
3. **`src/main.cpp`** - Firmware source (no `.ino` required)
4. **This file** - Quick start guide

## Quick Start (Windows)

### Step 1: Open in VS Code
```
1. Open VS Code
2. Open folder: D:\Projects\Clang\curser\SpeedLimiter\SPEED_LIMITER
3. Install "PlatformIO IDE" extension if not already installed
```

### Step 2: Build
```
pio run
```

### Step 3: Upload
```
pio run --target upload
```

### Step 4: Monitor
```
pio device monitor
```

## Quick Start (Linux/Mac)

### Step 1: Open in VS Code
```bash
cd /path/to/SPEED_LIMITER
code .
```

### Step 2: Install PlatformIO CLI
```bash
pip install platformio
```

### Step 3: Build
```bash
pio run
```

### Step 4: Upload
```bash
pio run --target upload
```

### Step 5: Monitor
```bash
pio device monitor
```

## VS Code PlatformIO Extension

### Build Task
1. Press `Ctrl+Shift+P`
2. Type: "Tasks: Run Task"
3. Select: "PlatformIO: Build"
4. Choose environment (default: esp32-wrover-n4r8-pwm)

### Upload Task
1. Press `Ctrl+Shift+P`
2. Type: "Tasks: Run Task"
3. Select: "PlatformIO: Upload"
4. Choose environment (default: esp32-wrover-n4r8-pwm)

### Monitor Task
1. Press `Ctrl+Shift+P`
2. Type: "Tasks: Run Task"
3. Select: "PlatformIO: Serial Monitor"
4. Choose port (auto-detected)

## Choosing Output Method

### PWM Mode (Default - Recommended)
```bash
pio run
```
- **Pins**: IO21 (SDPS1), IO22 (SDPS2)
- **Hardware**: Add RC filter (1kΩ + 10µF)
- **Resolution**: 10-bit (higher precision)
- **Best for**: Automotive ECU signals

### DAC Mode (Optional)
```bash
pio run -e esp32-wrover-n4r8-dac
```
- **Pins**: IO25 (SDPS1), IO26 (SDPS2)
- **Hardware**: Direct connection (no filter)
- **Resolution**: 8-bit (lower precision)
- **Best for**: True analog needs, space-constrained

## Troubleshooting

### Build Errors

**"PlatformIO not found"**
```bash
# Install PlatformIO extension in VS Code
# Or install CLI: pip install platformio
```

**"Compilation failed"**
```bash
# Update ESP32 platform
pio platform update

# Check for syntax errors
pio check
```

**"Library not found"**
```bash
# Install missing library
pio lib install esp32-can
pio lib install Preferences
```

### Upload Errors

**"Failed to connect"**
```
1. Disconnect serial monitor
2. Press BOOT button on ESP32
3. Try uploading again
4. Check USB cable (data cable, not charge-only)
```

**"Write timeout"**
```
1. Check serial port (correct COM port?)
2. Reduce upload speed: pio run --target upload --upload-speed 460800
3. Press BOOT button
```

## Next Steps

1. Read `README_PlatformIO.md` for detailed information
2. Review hardware requirements for your chosen mode
3. Connect hardware (PWM filters or direct DAC)
4. Upload firmware
5. Use CLI commands to configure system
6. Test with procedure in `NewFirmware_Test_Procedure_Detailed.txt`

## Need Help?

- **Documentation**: See all project .txt and .md files
- **PlatformIO Docs**: https://docs.platformio.org/
- **ESP32 Docs**: https://docs.espressif.com/projects/arduino-esp32
- **Issues**: Check pin connections, hardware components, serial port

---

**Happy Building! 🚀**

