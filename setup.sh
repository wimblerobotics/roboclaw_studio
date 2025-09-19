#!/bin/bash
# Setup script for RoboClaw Motion Studio

echo "Setting up RoboClaw Motion Studio..."

# Make sure we're in the right directory
cd "$(dirname "$0")"

# Install desktop entry
echo "Installing desktop entry..."
mkdir -p ~/.local/share/applications
cp roboclaw-motion-studio.desktop ~/.local/share/applications/
chmod +x ~/.local/share/applications/roboclaw-motion-studio.desktop

# Update desktop database
if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database ~/.local/share/applications
    echo "Desktop database updated"
fi

# Set up udev rules for RoboClaw devices (requires sudo)
echo "Setting up udev rules for RoboClaw devices..."
if [ "$EUID" -eq 0 ]; then
    cat > /etc/udev/rules.d/99-roboclaw.rules << 'EOF'
# RoboClaw motor controller devices
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="dialout", MODE="0666", SYMLINK+="roboclaw%n"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", GROUP="dialout", MODE="0666", SYMLINK+="roboclaw%n"
# Generic USB-Serial adapters commonly used with RoboClaw
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", GROUP="dialout", MODE="0666"
EOF
    udevadm control --reload-rules
    udevadm trigger
    echo "Udev rules installed. RoboClaw devices will be accessible without sudo."
else
    echo "Note: To access RoboClaw devices without sudo, run this script as root or add yourself to the dialout group:"
    echo "  sudo usermod -a -G dialout $USER"
    echo "  (Then log out and back in)"
fi

# Create symbolic link for easy command-line access
echo "Creating command-line launcher..."
mkdir -p ~/.local/bin
cat > ~/.local/bin/roboclaw-studio << 'EOF'
#!/bin/bash
cd /home/ros/motion_studio_ws
# Activate virtual environment and run the application
source venv/bin/activate
python main.py "$@"
EOF
chmod +x ~/.local/bin/roboclaw-studio

# Add ~/.local/bin to PATH if not already there
if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    echo "Added ~/.local/bin to PATH in ~/.bashrc"
    echo "Run 'source ~/.bashrc' or restart your terminal to use 'roboclaw-studio' command"
fi

echo ""
echo "Setup complete!"
echo ""
echo "You can now:"
echo "1. Find 'RoboClaw Motion Studio' in your application menu"
echo "2. Run 'roboclaw-studio' from the command line (after restarting terminal)"
echo "3. Pin the application to your taskbar for easy access"
echo ""
echo "For first-time setup:"
echo "1. Connect your RoboClaw device via USB"
echo "2. Start the application"
echo "3. Go to Connection tab and select your device"
echo "4. Follow the connection instructions in the app"
