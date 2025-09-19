"""
Device connection tab for RoboClaw Motion Studio
"""

import os
import glob
import logging
from typing import Optional, List
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                            QComboBox, QPushButton, QLabel, QSpinBox,
                            QMessageBox, QProgressBar, QTextEdit)
from PyQt6.QtCore import QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont

from .roboclaw_linux import RoboClawLinux

logger = logging.getLogger(__name__)

class ConnectionWorker(QThread):
    """Worker thread for device connection"""
    
    connection_result = pyqtSignal(bool, str)  # success, message
    
    def __init__(self, port: str, baudrate: int):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
    
    def run(self):
        """Attempt to connect to RoboClaw device"""
        try:
            roboclaw = RoboClawLinux(self.port)
            if roboclaw.connect(self.baudrate):
                # Test communication
                version = roboclaw.get_version()
                if version:
                    self.connection_result.emit(True, f"Connected to {version}")
                    return
                else:
                    roboclaw.disconnect()
                    self.connection_result.emit(False, "Device not responding")
            else:
                self.connection_result.emit(False, "Failed to open serial port")
        except Exception as e:
            self.connection_result.emit(False, f"Connection error: {e}")

class DeviceConnectionTab(QWidget):
    """Device connection and communication settings tab"""
    
    device_connected = pyqtSignal(RoboClawLinux)
    device_disconnected = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.roboclaw: Optional[RoboClawLinux] = None
        self.connection_worker: Optional[ConnectionWorker] = None
        
        self._create_ui()
        self._refresh_ports()
        
        # Auto-refresh ports timer
        self.port_refresh_timer = QTimer()
        self.port_refresh_timer.timeout.connect(self._refresh_ports)
        self.port_refresh_timer.start(2000)  # Refresh every 2 seconds
    
    def _create_ui(self):
        """Create user interface"""
        layout = QVBoxLayout(self)
        
        # Connection settings group
        connection_group = QGroupBox("Connection Settings")
        connection_layout = QVBoxLayout(connection_group)
        
        # Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        port_layout.addWidget(self.port_combo)
        
        self.refresh_ports_btn = QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self._refresh_ports)
        port_layout.addWidget(self.refresh_ports_btn)
        
        port_layout.addStretch()
        connection_layout.addLayout(port_layout)
        
        # Baud rate selection
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems([
            "2400", "4800", "9600", "19200", "38400", 
            "57600", "115200", "230400", "460800"
        ])
        self.baud_combo.setCurrentText("38400")  # Default for RoboClaw
        baud_layout.addWidget(self.baud_combo)
        baud_layout.addStretch()
        connection_layout.addLayout(baud_layout)
        
        # Device address
        addr_layout = QHBoxLayout()
        addr_layout.addWidget(QLabel("Device Address:"))
        self.address_spin = QSpinBox()
        self.address_spin.setRange(0x80, 0x87)  # Standard RoboClaw address range
        self.address_spin.setValue(0x80)
        self.address_spin.setDisplayIntegerBase(16)
        self.address_spin.setPrefix("0x")
        addr_layout.addWidget(self.address_spin)
        addr_layout.addStretch()
        connection_layout.addLayout(addr_layout)
        
        # Connection buttons
        button_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_device)
        button_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_device)
        self.disconnect_btn.setEnabled(False)
        button_layout.addWidget(self.disconnect_btn)
        
        button_layout.addStretch()
        connection_layout.addLayout(button_layout)
        
        # Connection progress
        self.connection_progress = QProgressBar()
        self.connection_progress.setVisible(False)
        connection_layout.addWidget(self.connection_progress)
        
        layout.addWidget(connection_group)
        
        # Device information group
        device_info_group = QGroupBox("Device Information")
        device_info_layout = QVBoxLayout(device_info_group)
        
        self.device_info_text = QTextEdit()
        self.device_info_text.setMaximumHeight(150)
        self.device_info_text.setReadOnly(True)
        font = QFont("Courier")
        font.setPointSize(10)
        self.device_info_text.setFont(font)
        device_info_layout.addWidget(self.device_info_text)
        
        layout.addWidget(device_info_group)
        
        # Instructions
        instructions_group = QGroupBox("Instructions")
        instructions_layout = QVBoxLayout(instructions_group)
        
        instructions_text = QLabel("""
        <h4>Connection Instructions:</h4>
        <ol>
        <li>Connect your RoboClaw device to the computer via USB</li>
        <li>Select the appropriate serial port from the dropdown</li>
        <li>Choose the correct baud rate (default is 38400)</li>
        <li>Set the device address (default is 0x80)</li>
        <li>Click "Connect" to establish communication</li>
        </ol>
        
        <h4>Supported Devices:</h4>
        <ul>
        <li>RoboClaw 2x7A</li>
        <li>RoboClaw 2x15A</li>
        <li>RoboClaw 2x30A</li>
        <li>RoboClaw 2x45A</li>
        <li>RoboClaw 2x60A</li>
        <li>And other compatible models</li>
        </ul>
        """)
        instructions_text.setWordWrap(True)
        instructions_layout.addWidget(instructions_text)
        
        layout.addWidget(instructions_group)
        
        layout.addStretch()
    
    def _refresh_ports(self):
        """Refresh available serial ports"""
        current_port = self.port_combo.currentText()
        self.port_combo.clear()
        
        # Get available ports
        ports = self._get_available_ports()
        
        if ports:
            self.port_combo.addItems(ports)
            # Try to restore previous selection
            if current_port in ports:
                self.port_combo.setCurrentText(current_port)
        else:
            self.port_combo.addItem("No ports available")
    
    def _get_available_ports(self) -> List[str]:
        """Get list of available serial ports on Linux"""
        ports = []
        
        # Common USB serial device patterns
        patterns = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
            '/dev/ttyS*'
        ]
        
        for pattern in patterns:
            ports.extend(glob.glob(pattern))
        
        # Sort ports naturally
        ports.sort()
        
        return ports
    
    def connect_device(self):
        """Connect to RoboClaw device"""
        port = self.port_combo.currentText()
        
        if not port or port == "No ports available":
            QMessageBox.warning(self, "Warning", "Please select a valid serial port")
            return
        
        if not os.path.exists(port):
            QMessageBox.warning(self, "Warning", f"Serial port {port} does not exist")
            return
        
        try:
            baudrate = int(self.baud_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, "Warning", "Invalid baud rate")
            return
        
        # Disable UI during connection
        self.connect_btn.setEnabled(False)
        self.connection_progress.setVisible(True)
        self.connection_progress.setRange(0, 0)  # Indeterminate progress
        
        # Start connection in worker thread
        self.connection_worker = ConnectionWorker(port, baudrate)
        self.connection_worker.connection_result.connect(self._on_connection_result)
        self.connection_worker.finished.connect(self._on_connection_finished)
        self.connection_worker.start()
        
        logger.info(f"Attempting to connect to {port} at {baudrate} baud")
    
    def _on_connection_result(self, success: bool, message: str):
        """Handle connection attempt result"""
        if success:
            # Create RoboClaw instance
            port = self.port_combo.currentText()
            baudrate = int(self.baud_combo.currentText())
            
            self.roboclaw = RoboClawLinux(port)
            self.roboclaw.connect(baudrate)
            self.roboclaw.address = self.address_spin.value()
            
            # Update UI
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.port_combo.setEnabled(False)
            self.baud_combo.setEnabled(False)
            self.address_spin.setEnabled(False)
            
            # Get device information
            self._update_device_info()
            
            # Emit signal
            self.device_connected.emit(self.roboclaw)
            
            QMessageBox.information(self, "Success", message)
            logger.info(f"Connected successfully: {message}")
            
        else:
            QMessageBox.critical(self, "Connection Failed", message)
            logger.error(f"Connection failed: {message}")
    
    def _on_connection_finished(self):
        """Handle connection thread finished"""
        self.connection_progress.setVisible(False)
        if not self.disconnect_btn.isEnabled():  # Connection failed
            self.connect_btn.setEnabled(True)
    
    def disconnect_device(self):
        """Disconnect from RoboClaw device"""
        if self.roboclaw:
            self.roboclaw.disconnect()
            self.roboclaw = None
        
        # Update UI
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)
        self.address_spin.setEnabled(True)
        
        # Clear device info
        self.device_info_text.clear()
        
        # Emit signal
        self.device_disconnected.emit()
        
        logger.info("Disconnected from device")
    
    def _update_device_info(self):
        """Update device information display"""
        if not self.roboclaw:
            return
        
        info_lines = []
        
        try:
            # Firmware version
            version = self.roboclaw.get_version()
            if version:
                info_lines.append(f"Firmware Version: {version}")
            
            # Main battery voltage
            voltage = self.roboclaw.get_main_battery_voltage()
            if voltage is not None:
                info_lines.append(f"Main Battery: {voltage:.2f}V")
            
            # Logic battery voltage
            logic_voltage = self.roboclaw.get_logic_battery_voltage()
            if logic_voltage is not None:
                info_lines.append(f"Logic Battery: {logic_voltage:.2f}V")
            
            # Temperature
            temp = self.roboclaw.get_temperature()
            if temp is not None:
                info_lines.append(f"Temperature: {temp:.1f}°C")
            
            # Error status
            error = self.roboclaw.get_error_status()
            if error is not None:
                if error == 0:
                    info_lines.append("Error Status: No errors")
                else:
                    info_lines.append(f"Error Status: 0x{error:08X}")
            
            # Connection details
            info_lines.append(f"Port: {self.port_combo.currentText()}")
            info_lines.append(f"Baud Rate: {self.baud_combo.currentText()}")
            info_lines.append(f"Address: 0x{self.address_spin.value():02X}")
            
        except Exception as e:
            info_lines.append(f"Error reading device info: {e}")
            logger.error(f"Error reading device info: {e}")
        
        self.device_info_text.setPlainText("\n".join(info_lines))
    
    def get_roboclaw(self) -> Optional[RoboClawLinux]:
        """Get current RoboClaw instance"""
        return self.roboclaw
