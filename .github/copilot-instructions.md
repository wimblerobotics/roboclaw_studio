<!-- Use this file to provide workspace-specific custom instructions to Copilot. For more details, visit https://code.visualstudio.com/docs/copilot/copilot-customization#_use-a-githubcopilotinstructionsmd-file -->

# RoboClaw Motion Studio Clone Project

This is a Python PyQt6 GUI application that provides a comprehensive interface for controlling and monitoring RoboClaw motor controllers on Linux systems.

## Project Structure
- `main.py` - Application entry point
- `roboclaw_motion_studio/` - Main package directory
  - `main_window.py` - Main application window with tab management
  - `roboclaw_linux.py` - Linux-compatible RoboClaw communication library
  - `device_connection_tab.py` - Device connection and communication settings
  - `motor_control_tab.py` - Real-time motor control interface
  - `pid_tuning_tab.py` - PID tuning with auto-tuning algorithms
  - `monitoring_tab.py` - Real-time monitoring and data visualization
  - `configuration_tab.py` - Device configuration management

## Key Features
- Real-time motor monitoring (position, speed, current, temperature)
- PID tuning with auto-tuning algorithms (Step Response, Relay Feedback, Ziegler-Nichols)
- Device configuration management with save/load functionality
- Data visualization with matplotlib
- Emergency stop functionality
- Configuration backup and restore

## Development Guidelines
- Follow PyQt6 patterns and best practices
- Use proper error handling and logging
- Implement threaded operations for device communication
- Maintain separation of concerns between UI and business logic
- Use type hints throughout the codebase
- Follow PEP 8 style guidelines

## Dependencies
- PyQt6 for GUI framework
- pyserial for device communication
- matplotlib for data visualization
- numpy and scipy for data analysis and auto-tuning algorithms

## Auto-tuning Algorithms
The application implements three auto-tuning methods:
1. **Step Response** - Analyzes system response to step input using Cohen-Coon tuning rules
2. **Relay Feedback** - Uses relay oscillations to determine ultimate gain and period
3. **Ziegler-Nichols** - Classic method for PID parameter estimation

When working on auto-tuning features, ensure thread safety and provide user feedback during the tuning process.
