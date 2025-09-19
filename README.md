# RoboClaw Motion Studio Clone

A comprehensive Linux GUI application for controlling and monitoring RoboClaw motor controllers, built with Python and PyQt6.

## Features

- **Real-time Motor Control**: Direct control of motor speed and direction with safety features
- **PID Tuning with Auto-tuning**: Advanced PID parameter tuning with three auto-tuning algorithms:
  - Step Response Analysis (Cohen-Coon method)
  - Relay Feedback Method  
  - Ziegler-Nichols Method
- **Real-time Monitoring**: Live data visualization of motor parameters including:
  - Encoder positions and speeds
  - Motor currents and PWM values
  - Battery voltages and temperature
  - Error status monitoring
- **Device Configuration**: Complete device configuration management with save/load functionality
- **Data Visualization**: Interactive plots with configurable time ranges and data types
- **Emergency Stop**: Immediate motor shutdown with keyboard shortcut (Escape)
- **Configuration Backup**: Save and restore device configurations to/from JSON files

## Supported Devices

- RoboClaw 2x7A
- RoboClaw 2x15A  
- RoboClaw 2x30A
- RoboClaw 2x45A
- RoboClaw 2x60A
- Other compatible RoboClaw models

## Requirements

- Ubuntu 24.04 or compatible Linux distribution
- Python 3.12+
- USB connection to RoboClaw device

## Installation

1. Clone or download the project:
```bash
cd /home/ros/motion_studio_ws
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Run the application:
```bash
python main.py
```

## Usage

### Quick Start

1. **Connect Device**: 
   - Connect your RoboClaw to the computer via USB
   - Go to the "Connection" tab
   - Select the appropriate serial port (usually /dev/ttyUSB0 or /dev/ttyACM0)
   - Choose baud rate (default: 38400)
   - Click "Connect"

2. **Motor Control**:
   - Switch to "Motor Control" tab
   - Enable motors with the checkbox
   - Use sliders or buttons to control motor speed
   - Emergency stop available with ESC key

3. **PID Tuning**:
   - Go to "PID Tuning" tab
   - Read current PID values from device
   - Choose auto-tuning method (Step Response recommended)
   - Click "Start Auto-Tuning" and follow prompts
   - Apply suggested values or manually adjust

4. **Monitoring**:
   - Switch to "Monitoring" tab
   - Enable monitoring to see real-time data
   - Choose different plot types to visualize data
   - Monitor system health (voltage, temperature, errors)

5. **Configuration**:
   - Use "Configuration" tab to manage device settings
   - Save/load configurations to preserve settings
   - Backup configurations before making changes

### Auto-Tuning Guide

The application provides three auto-tuning methods:

#### Step Response Method (Recommended for most cases)
- Applies a step input to the motor
- Analyzes the response characteristics
- Uses Cohen-Coon tuning rules for stable performance
- Best for systems with clear step response

#### Relay Feedback Method
- Uses relay switching to induce oscillations  
- Determines ultimate gain and period
- Applies Ziegler-Nichols rules
- Good for systems that oscillate well

#### Ziegler-Nichols Method
- Classic PID tuning approach
- Finds critical gain that causes sustained oscillation
- Calculates PID parameters from oscillation characteristics

**Auto-tuning Tips:**
- Ensure motor can move freely before starting
- Start with Step Response method for initial tuning
- Fine-tune manually after auto-tuning if needed
- Save working configurations before experimenting

## Configuration Files

The application supports saving and loading configurations in JSON format. Configuration files include:

- Motor PID parameters
- Voltage thresholds  
- Current limits
- Communication settings
- Advanced device parameters

Example configuration structure:
```json
{
  "voltage_settings": {
    "main_battery": {"min_voltage": 10.5, "max_voltage": 16.0},
    "logic_battery": {"min_voltage": 4.0, "max_voltage": 5.5}
  },
  "motor_settings": {
    "motor1": {"max_current": 7.0, "default_accel": 2000},
    "motor2": {"max_current": 7.0, "default_accel": 2000}
  }
}
```

## Troubleshooting

### Connection Issues
- Check USB cable and connections
- Verify correct serial port selection
- Try different baud rates (38400, 115200, 19200)
- Check device permissions: `sudo chmod 666 /dev/ttyUSB0`

### Auto-tuning Problems
- Ensure motor can move freely
- Check encoder connections and operation
- Verify adequate power supply
- Start with lower speeds for safety

### Performance Issues  
- Reduce monitoring update rate
- Close other applications using serial ports
- Check for electromagnetic interference

## Desktop Integration

To make the application available in the application menu:

1. Create desktop entry:
```bash
mkdir -p ~/.local/share/applications
cat > ~/.local/share/applications/roboclaw-motion-studio.desktop << EOF
[Desktop Entry]
Name=RoboClaw Motion Studio
Comment=Motor controller interface for RoboClaw devices
Exec=python3 /home/ros/motion_studio_ws/main.py
Icon=applications-system
Terminal=false
Type=Application
Categories=Development;Engineering;
EOF
```

2. Update desktop database:
```bash
update-desktop-database ~/.local/share/applications
```

## Development

### Project Structure
```
motion_studio_ws/
├── main.py                          # Application entry point
├── requirements.txt                 # Python dependencies
├── roboclaw_motion_studio/         # Main package
│   ├── __init__.py
│   ├── main_window.py              # Main application window
│   ├── roboclaw_linux.py           # Device communication
│   ├── device_connection_tab.py    # Connection interface
│   ├── motor_control_tab.py        # Motor control
│   ├── pid_tuning_tab.py           # PID tuning with auto-tune
│   ├── monitoring_tab.py           # Data monitoring/visualization
│   └── configuration_tab.py        # Configuration management
└── .github/
    └── copilot-instructions.md     # Development guidelines
```

### Contributing
- Follow PEP 8 style guidelines
- Add type hints for all functions
- Include proper error handling and logging
- Test with actual RoboClaw hardware when possible
- Document new features and changes

## License

This project is open source. Please refer to the original RoboClaw library license for any restrictions.

## Acknowledgments

- Based on the original RoboClaw Arduino library
- Uses PyQt6 for cross-platform GUI framework
- Implements established PID tuning algorithms from control theory literature
