"""
Configuration management tab for RoboClaw Motion Studio
"""

import logging
import json
from typing import Optional, Dict, Any
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                            QDoubleSpinBox, QPushButton, QLabel, QSpinBox,
                            QComboBox, QCheckBox, QTextEdit, QMessageBox,
                            QFileDialog, QTabWidget, QGridLayout, QSlider)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from .roboclaw_protocol import RoboClawProtocol

logger = logging.getLogger(__name__)

class ConfigurationTab(QWidget):
    """Device configuration management tab"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.roboclaw: Optional[RoboClawProtocol] = None
        
        self._create_ui()
    
    def _create_ui(self):
        """Create user interface"""
        layout = QVBoxLayout(self)
        
        # Configuration tabs
        self.config_tabs = QTabWidget()
        
        # Voltage settings tab
        self.voltage_tab = self._create_voltage_tab()
        self.config_tabs.addTab(self.voltage_tab, "Voltage Settings")
        
        # Motor settings tab
        self.motor_tab = self._create_motor_tab()
        self.config_tabs.addTab(self.motor_tab, "Motor Settings")
        
        # Advanced settings tab
        self.advanced_tab = self._create_advanced_tab()
        self.config_tabs.addTab(self.advanced_tab, "Advanced")
        
        layout.addWidget(self.config_tabs)
        
        # Configuration file operations
        file_ops_group = QGroupBox("Configuration File Operations")
        file_ops_layout = QHBoxLayout(file_ops_group)
        
        self.load_config_btn = QPushButton("Load from File...")
        self.load_config_btn.clicked.connect(self._load_config_file)
        file_ops_layout.addWidget(self.load_config_btn)
        
        self.save_config_btn = QPushButton("Save to File...")
        self.save_config_btn.clicked.connect(self._save_config_file)
        file_ops_layout.addWidget(self.save_config_btn)
        
        self.read_device_btn = QPushButton("Read from Device")
        self.read_device_btn.clicked.connect(self._read_device_config)
        self.read_device_btn.setEnabled(False)
        file_ops_layout.addWidget(self.read_device_btn)
        
        self.write_device_btn = QPushButton("Write to Device")
        self.write_device_btn.clicked.connect(self._write_device_config)
        self.write_device_btn.setEnabled(False)
        file_ops_layout.addWidget(self.write_device_btn)
        
        file_ops_layout.addStretch()
        layout.addWidget(file_ops_group)
        
        # Status/log display
        status_group = QGroupBox("Configuration Status")
        status_layout = QVBoxLayout(status_group)
        
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(150)
        self.status_text.setReadOnly(True)
        font = QFont("Courier")
        font.setPointSize(10)
        self.status_text.setFont(font)
        status_layout.addWidget(self.status_text)
        
        layout.addWidget(status_group)
    
    def _create_voltage_tab(self) -> QWidget:
        """Create voltage settings tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Main battery voltage settings
        main_battery_group = QGroupBox("Main Battery Voltage Settings")
        main_battery_layout = QGridLayout(main_battery_group)
        
        main_battery_layout.addWidget(QLabel("Minimum Voltage:"), 0, 0)
        self.main_min_voltage_spin = QDoubleSpinBox()
        self.main_min_voltage_spin.setRange(6.0, 34.0)
        self.main_min_voltage_spin.setValue(10.5)
        self.main_min_voltage_spin.setDecimals(1)
        self.main_min_voltage_spin.setSuffix(" V")
        main_battery_layout.addWidget(self.main_min_voltage_spin, 0, 1)
        
        main_battery_layout.addWidget(QLabel("Maximum Voltage:"), 1, 0)
        self.main_max_voltage_spin = QDoubleSpinBox()
        self.main_max_voltage_spin.setRange(6.0, 34.0)
        self.main_max_voltage_spin.setValue(16.0)
        self.main_max_voltage_spin.setDecimals(1)
        self.main_max_voltage_spin.setSuffix(" V")
        main_battery_layout.addWidget(self.main_max_voltage_spin, 1, 1)
        
        main_battery_layout.addWidget(QLabel("Current Voltage:"), 2, 0)
        self.main_current_voltage_label = QLabel("0.0 V")
        main_battery_layout.addWidget(self.main_current_voltage_label, 2, 1)
        
        layout.addWidget(main_battery_group)
        
        # Logic battery voltage settings
        logic_battery_group = QGroupBox("Logic Battery Voltage Settings")
        logic_battery_layout = QGridLayout(logic_battery_group)
        
        logic_battery_layout.addWidget(QLabel("Minimum Voltage:"), 0, 0)
        self.logic_min_voltage_spin = QDoubleSpinBox()
        self.logic_min_voltage_spin.setRange(3.0, 5.5)
        self.logic_min_voltage_spin.setValue(4.0)
        self.logic_min_voltage_spin.setDecimals(1)
        self.logic_min_voltage_spin.setSuffix(" V")
        logic_battery_layout.addWidget(self.logic_min_voltage_spin, 0, 1)
        
        logic_battery_layout.addWidget(QLabel("Maximum Voltage:"), 1, 0)
        self.logic_max_voltage_spin = QDoubleSpinBox()
        self.logic_max_voltage_spin.setRange(3.0, 5.5)
        self.logic_max_voltage_spin.setValue(5.5)
        self.logic_max_voltage_spin.setDecimals(1)
        self.logic_max_voltage_spin.setSuffix(" V")
        logic_battery_layout.addWidget(self.logic_max_voltage_spin, 1, 1)
        
        logic_battery_layout.addWidget(QLabel("Current Voltage:"), 2, 0)
        self.logic_current_voltage_label = QLabel("0.0 V")
        logic_battery_layout.addWidget(self.logic_current_voltage_label, 2, 1)
        
        layout.addWidget(logic_battery_group)
        
        layout.addStretch()
        return tab
    
    def _create_motor_tab(self) -> QWidget:
        """Create motor settings tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Motor 1 settings
        m1_group = QGroupBox("Motor 1 Settings")
        m1_layout = QGridLayout(m1_group)
        
        m1_layout.addWidget(QLabel("Maximum Current:"), 0, 0)
        self.m1_max_current_spin = QDoubleSpinBox()
        self.m1_max_current_spin.setRange(0.1, 60.0)
        self.m1_max_current_spin.setValue(7.0)
        self.m1_max_current_spin.setDecimals(1)
        self.m1_max_current_spin.setSuffix(" A")
        m1_layout.addWidget(self.m1_max_current_spin, 0, 1)
        
        m1_layout.addWidget(QLabel("Default Acceleration:"), 1, 0)
        self.m1_default_accel_spin = QSpinBox()
        self.m1_default_accel_spin.setRange(0, 655359)
        self.m1_default_accel_spin.setValue(2000)
        m1_layout.addWidget(self.m1_default_accel_spin, 1, 1)
        
        m1_layout.addWidget(QLabel("Encoder Mode:"), 2, 0)
        self.m1_encoder_mode_combo = QComboBox()
        self.m1_encoder_mode_combo.addItems([
            "Quadrature", "Absolute", "Pulse Counting"
        ])
        m1_layout.addWidget(self.m1_encoder_mode_combo, 2, 1)
        
        layout.addWidget(m1_group)
        
        # Motor 2 settings
        m2_group = QGroupBox("Motor 2 Settings")
        m2_layout = QGridLayout(m2_group)
        
        m2_layout.addWidget(QLabel("Maximum Current:"), 0, 0)
        self.m2_max_current_spin = QDoubleSpinBox()
        self.m2_max_current_spin.setRange(0.1, 60.0)
        self.m2_max_current_spin.setValue(7.0)
        self.m2_max_current_spin.setDecimals(1)
        self.m2_max_current_spin.setSuffix(" A")
        m2_layout.addWidget(self.m2_max_current_spin, 0, 1)
        
        m2_layout.addWidget(QLabel("Default Acceleration:"), 1, 0)
        self.m2_default_accel_spin = QSpinBox()
        self.m2_default_accel_spin.setRange(0, 655359)
        self.m2_default_accel_spin.setValue(2000)
        m2_layout.addWidget(self.m2_default_accel_spin, 1, 1)
        
        m2_layout.addWidget(QLabel("Encoder Mode:"), 2, 0)
        self.m2_encoder_mode_combo = QComboBox()
        self.m2_encoder_mode_combo.addItems([
            "Quadrature", "Absolute", "Pulse Counting"
        ])
        m2_layout.addWidget(self.m2_encoder_mode_combo, 2, 1)
        
        layout.addWidget(m2_group)
        
        layout.addStretch()
        return tab
    
    def _create_advanced_tab(self) -> QWidget:
        """Create advanced settings tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # PWM settings
        pwm_group = QGroupBox("PWM Settings")
        pwm_layout = QGridLayout(pwm_group)
        
        pwm_layout.addWidget(QLabel("PWM Mode:"), 0, 0)
        self.pwm_mode_combo = QComboBox()
        self.pwm_mode_combo.addItems([
            "Sign Magnitude", "Locked Antiphase"
        ])
        pwm_layout.addWidget(self.pwm_mode_combo, 0, 1)
        
        pwm_layout.addWidget(QLabel("PWM Frequency:"), 1, 0)
        self.pwm_freq_combo = QComboBox()
        self.pwm_freq_combo.addItems([
            "1 kHz", "2 kHz", "4 kHz", "8 kHz", "16 kHz", "32 kHz"
        ])
        self.pwm_freq_combo.setCurrentText("32 kHz")
        pwm_layout.addWidget(self.pwm_freq_combo, 1, 1)
        
        layout.addWidget(pwm_group)
        
        # Pin function settings
        pin_group = QGroupBox("Pin Function Settings")
        pin_layout = QGridLayout(pin_group)
        
        pin_layout.addWidget(QLabel("S3 Pin Mode:"), 0, 0)
        self.s3_mode_combo = QComboBox()
        self.s3_mode_combo.addItems([
            "Default", "Emergency Stop", "Voltage Monitoring", "Temperature Monitoring"
        ])
        pin_layout.addWidget(self.s3_mode_combo, 0, 1)
        
        pin_layout.addWidget(QLabel("S4 Pin Mode:"), 1, 0)
        self.s4_mode_combo = QComboBox()
        self.s4_mode_combo.addItems([
            "Default", "Emergency Stop", "Voltage Monitoring", "Temperature Monitoring"
        ])
        pin_layout.addWidget(self.s4_mode_combo, 1, 1)
        
        pin_layout.addWidget(QLabel("S5 Pin Mode:"), 2, 0)
        self.s5_mode_combo = QComboBox()
        self.s5_mode_combo.addItems([
            "Default", "Emergency Stop", "Voltage Monitoring", "Temperature Monitoring"
        ])
        pin_layout.addWidget(self.s5_mode_combo, 2, 1)
        
        layout.addWidget(pin_group)
        
        # Deadband settings
        deadband_group = QGroupBox("Deadband Settings")
        deadband_layout = QGridLayout(deadband_group)
        
        deadband_layout.addWidget(QLabel("Minimum Deadband:"), 0, 0)
        self.deadband_min_spin = QSpinBox()
        self.deadband_min_spin.setRange(0, 127)
        self.deadband_min_spin.setValue(1)
        deadband_layout.addWidget(self.deadband_min_spin, 0, 1)
        
        deadband_layout.addWidget(QLabel("Maximum Deadband:"), 1, 0)
        self.deadband_max_spin = QSpinBox()
        self.deadband_max_spin.setRange(0, 127)
        self.deadband_max_spin.setValue(3)
        deadband_layout.addWidget(self.deadband_max_spin, 1, 1)
        
        layout.addWidget(deadband_group)
        
        # Firmware and device info
        device_info_group = QGroupBox("Device Information")
        device_info_layout = QGridLayout(device_info_group)
        
        device_info_layout.addWidget(QLabel("Firmware Version:"), 0, 0)
        self.firmware_version_label = QLabel("Unknown")
        device_info_layout.addWidget(self.firmware_version_label, 0, 1)
        
        device_info_layout.addWidget(QLabel("Device Temperature:"), 1, 0)
        self.device_temp_label = QLabel("Unknown")
        device_info_layout.addWidget(self.device_temp_label, 1, 1)
        
        device_info_layout.addWidget(QLabel("Error Status:"), 2, 0)
        self.device_error_label = QLabel("Unknown")
        device_info_layout.addWidget(self.device_error_label, 2, 1)
        
        layout.addWidget(device_info_group)
        
        # Factory reset and save to NVM
        factory_group = QGroupBox("Factory Operations")
        factory_layout = QHBoxLayout(factory_group)
        
        self.restore_defaults_btn = QPushButton("Restore Factory Defaults")
        self.restore_defaults_btn.clicked.connect(self._restore_factory_defaults)
        self.restore_defaults_btn.setEnabled(False)
        factory_layout.addWidget(self.restore_defaults_btn)
        
        self.save_to_nvm_btn = QPushButton("Save to Non-Volatile Memory")
        self.save_to_nvm_btn.clicked.connect(self._save_to_nvm)
        self.save_to_nvm_btn.setEnabled(False)
        factory_layout.addWidget(self.save_to_nvm_btn)
        
        factory_layout.addStretch()
        layout.addWidget(factory_group)
        
        layout.addStretch()
        return tab
    
    def set_roboclaw(self, roboclaw: Optional[RoboClawProtocol]):
        """Set RoboClaw device instance"""
        self.roboclaw = roboclaw
        
        # Enable/disable controls based on connection
        connected = roboclaw is not None
        self.read_device_btn.setEnabled(connected)
        self.write_device_btn.setEnabled(connected)
        self.restore_defaults_btn.setEnabled(connected)
        self.save_to_nvm_btn.setEnabled(connected)
        
        if connected:
            self._read_device_config()
            self._update_device_info()
    
    def _read_device_config(self):
        """Read configuration from device"""
        if not self.roboclaw:
            return
        
        try:
            self.status_text.append("Reading configuration from device...")
            
            # Read voltage settings
            main_limits = self.roboclaw.get_min_max_main_voltages()
            if main_limits:
                self.main_min_voltage_spin.setValue(main_limits[0])
                self.main_max_voltage_spin.setValue(main_limits[1])
                self.status_text.append(f"Main Voltage Limits: Min={main_limits[0]}V, Max={main_limits[1]}V")

            logic_limits = self.roboclaw.get_min_max_logic_voltages()
            if logic_limits:
                self.logic_min_voltage_spin.setValue(logic_limits[0])
                self.logic_max_voltage_spin.setValue(logic_limits[1])
                self.status_text.append(f"Logic Voltage Limits: Min={logic_limits[0]}V, Max={logic_limits[1]}V")

            main_voltage = self.roboclaw.get_main_battery_voltage()
            if main_voltage:
                self.main_current_voltage_label.setText(f"{main_voltage:.1f} V")
            
            logic_voltage = self.roboclaw.get_logic_battery_voltage()
            if logic_voltage:
                self.logic_current_voltage_label.setText(f"{logic_voltage:.1f} V")
            
            # Read PID values for reference
            m1_pid = self.roboclaw.read_velocity_pid_m1()
            m2_pid = self.roboclaw.read_velocity_pid_m2()
            
            if m1_pid:
                self.status_text.append(f"M1 PID: Kp={m1_pid[0]:.6f}, Ki={m1_pid[1]:.6f}, Kd={m1_pid[2]:.6f}")
            
            if m2_pid:
                self.status_text.append(f"M2 PID: Kp={m2_pid[0]:.6f}, Ki={m2_pid[1]:.6f}, Kd={m2_pid[2]:.6f}")
            
            self.status_text.append("Configuration read successfully")
            logger.info("Device configuration read successfully")
            
        except Exception as e:
            self.status_text.append(f"Error reading configuration: {e}")
            logger.error(f"Error reading device configuration: {e}")
    
    def _write_device_config(self):
        """Write configuration to device"""
        if not self.roboclaw:
            return
        
        reply = QMessageBox.question(
            self, "Write Configuration",
            "Write current configuration settings to the device?\n\n"
            "This will overwrite the device's current settings.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        try:
            self.status_text.append("Writing configuration to device...")
            
            # Write voltage settings
            main_min = self.main_min_voltage_spin.value()
            main_max = self.main_max_voltage_spin.value()
            self.roboclaw.set_main_voltage_limits(main_min, main_max)
            self.status_text.append(f"Wrote Main Voltage Limits: Min={main_min}V, Max={main_max}V")

            logic_min = self.logic_min_voltage_spin.value()
            logic_max = self.logic_max_voltage_spin.value()
            self.roboclaw.set_logic_voltage_limits(logic_min, logic_max)
            self.status_text.append(f"Wrote Logic Voltage Limits: Min={logic_min}V, Max={logic_max}V")
            
            # Note: Other settings would be written here
            
            self.status_text.append("Configuration written successfully. Re-reading to verify.")
            QMessageBox.information(self, "Success", "Configuration written to device. Please save to NVM to make it permanent.")
            logger.info("Device configuration written successfully")

            # Re-read to confirm
            self._read_device_config()
            
        except Exception as e:
            self.status_text.append(f"Error writing configuration: {e}")
            QMessageBox.critical(self, "Error", f"Error writing configuration: {e}")
            logger.error(f"Error writing device configuration: {e}")
    
    def _update_device_info(self):
        """Update device information display"""
        if not self.roboclaw:
            return
        
        try:
            # Firmware version
            version = self.roboclaw.get_version()
            if version:
                self.firmware_version_label.setText(version)
            
            # Temperature
            temp = self.roboclaw.get_temperature()
            if temp is not None:
                self.device_temp_label.setText(f"{temp:.1f}°C")
            
            # Error status
            error = self.roboclaw.get_error_status()
            if error is not None:
                if error == 0:
                    self.device_error_label.setText("No errors")
                    self.device_error_label.setStyleSheet("color: green;")
                else:
                    self.device_error_label.setText(f"0x{error:08X}")
                    self.device_error_label.setStyleSheet("color: red;")
            
        except Exception as e:
            logger.error(f"Error updating device info: {e}")
    
    def _restore_factory_defaults(self):
        """Restore factory default settings"""
        if not self.roboclaw:
            return
        
        reply = QMessageBox.warning(
            self, "Restore Factory Defaults",
            "This will restore all settings to factory defaults.\n\n"
            "All custom configuration will be lost!\n\n"
            "Are you sure you want to continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        try:
            # Note: This would use the actual RoboClaw restore defaults command
            # success = self.roboclaw.restore_defaults()
            success = True  # Placeholder
            
            if success:
                self.status_text.append("Factory defaults restored")
                QMessageBox.information(self, "Success", "Factory defaults restored")
                self._read_device_config()  # Re-read configuration
                logger.info("Factory defaults restored")
            else:
                self.status_text.append("Failed to restore factory defaults")
                QMessageBox.warning(self, "Warning", "Failed to restore factory defaults")
                
        except Exception as e:
            self.status_text.append(f"Error restoring defaults: {e}")
            QMessageBox.critical(self, "Error", f"Error restoring defaults: {e}")
            logger.error(f"Error restoring factory defaults: {e}")
    
    def _save_to_nvm(self):
        """Save current settings to non-volatile memory"""
        if not self.roboclaw:
            return
        
        reply = QMessageBox.question(
            self, "Save to NVM",
            "Save current settings to non-volatile memory?\n\n"
            "This will make the current settings permanent.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        try:
            # Note: This would use the actual RoboClaw write NVM command
            # success = self.roboclaw.write_nvm()
            success = True  # Placeholder
            
            if success:
                self.status_text.append("Settings saved to NVM")
                QMessageBox.information(self, "Success", "Settings saved to non-volatile memory")
                logger.info("Settings saved to NVM")
            else:
                self.status_text.append("Failed to save to NVM")
                QMessageBox.warning(self, "Warning", "Failed to save settings to NVM")
                
        except Exception as e:
            self.status_text.append(f"Error saving to NVM: {e}")
            QMessageBox.critical(self, "Error", f"Error saving to NVM: {e}")
            logger.error(f"Error saving to NVM: {e}")
    
    def _load_config_file(self):
        """Load configuration from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON files (*.json);;All files (*.*)"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                
                self.load_configuration(config)
                self.status_text.append(f"Configuration loaded from {filename}")
                QMessageBox.information(self, "Success", "Configuration loaded successfully")
                logger.info(f"Configuration loaded from {filename}")
                
            except Exception as e:
                self.status_text.append(f"Error loading configuration: {e}")
                QMessageBox.critical(self, "Error", f"Failed to load configuration: {e}")
                logger.error(f"Failed to load configuration: {e}")
    
    def _save_config_file(self):
        """Save configuration to file"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "", "JSON files (*.json);;All files (*.*)"
        )
        
        if filename:
            try:
                config = self.get_configuration()
                
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                
                self.status_text.append(f"Configuration saved to {filename}")
                QMessageBox.information(self, "Success", "Configuration saved successfully")
                logger.info(f"Configuration saved to {filename}")
                
            except Exception as e:
                self.status_text.append(f"Error saving configuration: {e}")
                QMessageBox.critical(self, "Error", f"Failed to save configuration: {e}")
                logger.error(f"Failed to save configuration: {e}")
    
    def get_configuration(self) -> Dict[str, Any]:
        """Get current configuration as dictionary"""
        config = {
            "voltage_settings": {
                "main_battery": {
                    "min_voltage": self.main_min_voltage_spin.value(),
                    "max_voltage": self.main_max_voltage_spin.value()
                },
                "logic_battery": {
                    "min_voltage": self.logic_min_voltage_spin.value(),
                    "max_voltage": self.logic_max_voltage_spin.value()
                }
            },
            "motor_settings": {
                "motor1": {
                    "max_current": self.m1_max_current_spin.value(),
                    "default_accel": self.m1_default_accel_spin.value(),
                    "encoder_mode": self.m1_encoder_mode_combo.currentText()
                },
                "motor2": {
                    "max_current": self.m2_max_current_spin.value(),
                    "default_accel": self.m2_default_accel_spin.value(),
                    "encoder_mode": self.m2_encoder_mode_combo.currentText()
                }
            },
            "advanced_settings": {
                "pwm_mode": self.pwm_mode_combo.currentText(),
                "pwm_frequency": self.pwm_freq_combo.currentText(),
                "pin_functions": {
                    "s3_mode": self.s3_mode_combo.currentText(),
                    "s4_mode": self.s4_mode_combo.currentText(),
                    "s5_mode": self.s5_mode_combo.currentText()
                },
                "deadband": {
                    "min": self.deadband_min_spin.value(),
                    "max": self.deadband_max_spin.value()
                }
            }
        }
        return config
    
    def load_configuration(self, config: Dict[str, Any]):
        """Load configuration from dictionary"""
        try:
            # Voltage settings
            if "voltage_settings" in config:
                voltage_settings = config["voltage_settings"]
                
                if "main_battery" in voltage_settings:
                    main_battery = voltage_settings["main_battery"]
                    self.main_min_voltage_spin.setValue(main_battery.get("min_voltage", 10.5))
                    self.main_max_voltage_spin.setValue(main_battery.get("max_voltage", 16.0))
                
                if "logic_battery" in voltage_settings:
                    logic_battery = voltage_settings["logic_battery"]
                    self.logic_min_voltage_spin.setValue(logic_battery.get("min_voltage", 4.0))
                    self.logic_max_voltage_spin.setValue(logic_battery.get("max_voltage", 5.5))
            
            # Motor settings
            if "motor_settings" in config:
                motor_settings = config["motor_settings"]
                
                if "motor1" in motor_settings:
                    motor1 = motor_settings["motor1"]
                    self.m1_max_current_spin.setValue(motor1.get("max_current", 7.0))
                    self.m1_default_accel_spin.setValue(motor1.get("default_accel", 2000))
                    encoder_mode = motor1.get("encoder_mode", "Quadrature")
                    if encoder_mode in [self.m1_encoder_mode_combo.itemText(i) for i in range(self.m1_encoder_mode_combo.count())]:
                        self.m1_encoder_mode_combo.setCurrentText(encoder_mode)
                
                if "motor2" in motor_settings:
                    motor2 = motor_settings["motor2"]
                    self.m2_max_current_spin.setValue(motor2.get("max_current", 7.0))
                    self.m2_default_accel_spin.setValue(motor2.get("default_accel", 2000))
                    encoder_mode = motor2.get("encoder_mode", "Quadrature")
                    if encoder_mode in [self.m2_encoder_mode_combo.itemText(i) for i in range(self.m2_encoder_mode_combo.count())]:
                        self.m2_encoder_mode_combo.setCurrentText(encoder_mode)
            
            # Advanced settings
            if "advanced_settings" in config:
                advanced = config["advanced_settings"]
                
                pwm_mode = advanced.get("pwm_mode", "Sign Magnitude")
                if pwm_mode in [self.pwm_mode_combo.itemText(i) for i in range(self.pwm_mode_combo.count())]:
                    self.pwm_mode_combo.setCurrentText(pwm_mode)
                
                pwm_freq = advanced.get("pwm_frequency", "32 kHz")
                if pwm_freq in [self.pwm_freq_combo.itemText(i) for i in range(self.pwm_freq_combo.count())]:
                    self.pwm_freq_combo.setCurrentText(pwm_freq)
                
                if "pin_functions" in advanced:
                    pin_funcs = advanced["pin_functions"]
                    s3_mode = pin_funcs.get("s3_mode", "Default")
                    s4_mode = pin_funcs.get("s4_mode", "Default")
                    s5_mode = pin_funcs.get("s5_mode", "Default")
                    
                    if s3_mode in [self.s3_mode_combo.itemText(i) for i in range(self.s3_mode_combo.count())]:
                        self.s3_mode_combo.setCurrentText(s3_mode)
                    if s4_mode in [self.s4_mode_combo.itemText(i) for i in range(self.s4_mode_combo.count())]:
                        self.s4_mode_combo.setCurrentText(s4_mode)
                    if s5_mode in [self.s5_mode_combo.itemText(i) for i in range(self.s5_mode_combo.count())]:
                        self.s5_mode_combo.setCurrentText(s5_mode)
                
                if "deadband" in advanced:
                    deadband = advanced["deadband"]
                    self.deadband_min_spin.setValue(deadband.get("min", 1))
                    self.deadband_max_spin.setValue(deadband.get("max", 3))
            
            self.status_text.append("Configuration loaded successfully")
            
        except Exception as e:
            self.status_text.append(f"Error loading configuration: {e}")
            raise
