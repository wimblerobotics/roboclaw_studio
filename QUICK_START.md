# RoboClaw Motion Studio - Quick Start Guide

## 🚀 Quick Start

### 1. Launch the Application
```bash
# Option 1: Direct Python execution
python main.py

# Option 2: Using the installed command (after terminal restart)
roboclaw-studio

# Option 3: From the application menu
# Search for "RoboClaw Motion Studio"
```

### 2. Connect Your Device
1. Connect RoboClaw to computer via USB
2. Open "Connection" tab
3. Select serial port (usually `/dev/ttyUSB0` or `/dev/ttyACM0`)
4. Choose baud rate (default: 38400)
5. Click "Connect"

### 3. Basic Motor Control
1. Go to "Motor Control" tab
2. Check "Enable Motors"
3. Use sliders or buttons to control speed
4. **Emergency Stop**: Press `ESC` key anytime

## 🔧 PID Auto-Tuning Guide

### Step Response Method (Recommended)
Best for most applications:
1. Select "Step Response" method
2. Choose motor to tune
3. Ensure motor can move freely
4. Click "Start Auto-Tuning"
5. Wait for analysis (30-60 seconds)
6. Review and apply suggested values

### Relay Feedback Method
Good for systems that oscillate well:
1. Select "Relay Feedback" method
2. Ensure adequate space for oscillations
3. Monitor the tuning process
4. Apply or fine-tune results

### Manual Tuning Tips
- Start with Kp, set Ki=0, Kd=0
- Increase Kp until steady oscillation
- Add Ki to eliminate steady-state error
- Add Kd to reduce overshoot

## 📊 Monitoring Features

### Real-time Data
- Motor positions and speeds
- Current consumption
- Battery voltages
- Temperature monitoring
- Error status

### Data Visualization
- Choose plot type (position, speed, current, etc.)
- Adjust time range (10-300 seconds)
- Auto-scaling or fixed Y-axis
- Export data for analysis

## ⚙️ Configuration Management

### Save/Load Configurations
```json
{
  "voltage_settings": {
    "main_battery": {"min_voltage": 10.5, "max_voltage": 16.0}
  },
  "motor_settings": {
    "motor1": {"max_current": 7.0, "default_accel": 2000}
  }
}
```

### Device Settings
- Voltage thresholds
- Current limits
- PWM modes
- Pin functions
- Deadband settings

## 🛠️ Troubleshooting

### Connection Issues
```bash
# Check device permissions
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group
sudo usermod -a -G dialout $USER
# (logout and login again)

# Check if device is detected
dmesg | tail | grep tty
```

### Common Solutions
- **No response**: Check baud rate and address
- **Permission denied**: Run setup script as root or add to dialout group
- **Auto-tune fails**: Ensure motor can move freely, check encoder connections
- **High current**: Check motor connections and load

### Debug Mode
Enable debug mode in View menu for detailed logging.

## 📋 Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+C` | Connect Device |
| `Ctrl+D` | Disconnect Device |
| `Ctrl+O` | Load Configuration |
| `Ctrl+S` | Save Configuration |
| `Ctrl+Q` | Exit Application |
| `Escape` | Emergency Stop |

## 🔍 Advanced Features

### Auto-tuning Parameters
- **Step Response**: Uses Cohen-Coon tuning rules
- **Relay Feedback**: Implements Ziegler-Nichols method
- **Analysis**: Real-time system identification

### Safety Features
- Emergency stop (ESC key)
- Current limiting
- Voltage monitoring
- Temperature protection
- Error status monitoring

### Data Logging
- Enable in Monitoring tab
- Choose log file location
- CSV format for analysis
- Configurable sample rates

## 🎯 Best Practices

### Initial Setup
1. Test connection without load
2. Verify encoder operation
3. Set appropriate current limits
4. Tune PID with light load first
5. Save working configuration

### Tuning Process
1. Start with conservative values
2. Test in safe environment
3. Gradually increase performance
4. Document working parameters
5. Create backup configurations

### Safety Guidelines
- Always have emergency stop ready
- Test with light loads first
- Monitor temperature and current
- Use appropriate supply voltage
- Secure all connections

## 📞 Support

For issues and improvements:
1. Check the troubleshooting section
2. Enable debug mode for detailed logs
3. Verify hardware connections
4. Test with minimal configuration

## 🎉 Tips for Success

### Motor Tuning
- **Kp too high**: Oscillation, instability
- **Kp too low**: Slow response, large errors
- **Ki too high**: Overshoot, long settling
- **Ki too low**: Steady-state error
- **Kd too high**: Noise sensitivity
- **Kd too low**: Overshoot

### Performance Optimization
- Use appropriate QPPS values
- Monitor system performance
- Regular configuration backups
- Update firmware when available

Enjoy using RoboClaw Motion Studio! 🎊
