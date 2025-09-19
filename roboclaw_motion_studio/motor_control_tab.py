"""
Motor control tab for RoboClaw Motion Studio
"""

import logging
from typing import Optional
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                            QSlider, QPushButton, QLabel, QSpinBox,
                            QCheckBox, QProgressBar, QFrame)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPalette

from .roboclaw_linux import RoboClawLinux

logger = logging.getLogger(__name__)

class MotorControlTab(QWidget):
    """Motor control interface tab"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.roboclaw: Optional[RoboClawLinux] = None
        
        self._create_ui()
        self._setup_timers()
        
        # Control state
        self.motor1_speed = 0
        self.motor2_speed = 0
        self.motors_enabled = False
    
    def _create_ui(self):
        """Create user interface"""
        layout = QVBoxLayout(self)
        
        # Motor enable/disable
        control_group = QGroupBox("Motor Control")
        control_layout = QVBoxLayout(control_group)
        
        # Enable/disable controls
        enable_layout = QHBoxLayout()
        self.enable_checkbox = QCheckBox("Enable Motors")
        self.enable_checkbox.stateChanged.connect(self._on_motors_toggled)
        enable_layout.addWidget(self.enable_checkbox)
        
        self.emergency_stop_btn = QPushButton("EMERGENCY STOP")
        self.emergency_stop_btn.setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; }")
        self.emergency_stop_btn.clicked.connect(self._on_emergency_stop)
        enable_layout.addWidget(self.emergency_stop_btn)
        
        enable_layout.addStretch()
        control_layout.addLayout(enable_layout)
        
        layout.addWidget(control_group)
        
        # Motor 1 control
        m1_group = QGroupBox("Motor 1 (M1)")
        m1_layout = QVBoxLayout(m1_group)
        
        # Speed control
        m1_speed_layout = QHBoxLayout()
        m1_speed_layout.addWidget(QLabel("Speed:"))
        
        self.m1_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.m1_speed_slider.setRange(-127, 127)
        self.m1_speed_slider.setValue(0)
        self.m1_speed_slider.valueChanged.connect(self._on_m1_speed_changed)
        self.m1_speed_slider.setEnabled(False)
        m1_speed_layout.addWidget(self.m1_speed_slider)
        
        self.m1_speed_spinbox = QSpinBox()
        self.m1_speed_spinbox.setRange(-127, 127)
        self.m1_speed_spinbox.setValue(0)
        self.m1_speed_spinbox.valueChanged.connect(self._on_m1_speed_spinbox_changed)
        self.m1_speed_spinbox.setEnabled(False)
        m1_speed_layout.addWidget(self.m1_speed_spinbox)
        
        m1_layout.addLayout(m1_speed_layout)
        
        # Direction buttons
        m1_dir_layout = QHBoxLayout()
        self.m1_forward_btn = QPushButton("Forward")
        self.m1_forward_btn.clicked.connect(lambda: self._set_m1_speed(50))
        self.m1_forward_btn.setEnabled(False)
        m1_dir_layout.addWidget(self.m1_forward_btn)
        
        self.m1_stop_btn = QPushButton("Stop")
        self.m1_stop_btn.clicked.connect(lambda: self._set_m1_speed(0))
        self.m1_stop_btn.setEnabled(False)
        m1_dir_layout.addWidget(self.m1_stop_btn)
        
        self.m1_backward_btn = QPushButton("Backward")
        self.m1_backward_btn.clicked.connect(lambda: self._set_m1_speed(-50))
        self.m1_backward_btn.setEnabled(False)
        m1_dir_layout.addWidget(self.m1_backward_btn)
        
        m1_layout.addLayout(m1_dir_layout)
        
        # Motor 1 status
        m1_status_layout = QHBoxLayout()
        m1_status_layout.addWidget(QLabel("Encoder:"))
        self.m1_encoder_label = QLabel("0")
        m1_status_layout.addWidget(self.m1_encoder_label)
        
        m1_status_layout.addWidget(QLabel("Speed:"))
        self.m1_speed_label = QLabel("0")
        m1_status_layout.addWidget(self.m1_speed_label)
        
        m1_status_layout.addWidget(QLabel("Current:"))
        self.m1_current_label = QLabel("0.0A")
        m1_status_layout.addWidget(self.m1_current_label)
        
        m1_status_layout.addStretch()
        m1_layout.addLayout(m1_status_layout)
        
        layout.addWidget(m1_group)
        
        # Motor 2 control
        m2_group = QGroupBox("Motor 2 (M2)")
        m2_layout = QVBoxLayout(m2_group)
        
        # Speed control
        m2_speed_layout = QHBoxLayout()
        m2_speed_layout.addWidget(QLabel("Speed:"))
        
        self.m2_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.m2_speed_slider.setRange(-127, 127)
        self.m2_speed_slider.setValue(0)
        self.m2_speed_slider.valueChanged.connect(self._on_m2_speed_changed)
        self.m2_speed_slider.setEnabled(False)
        m2_speed_layout.addWidget(self.m2_speed_slider)
        
        self.m2_speed_spinbox = QSpinBox()
        self.m2_speed_spinbox.setRange(-127, 127)
        self.m2_speed_spinbox.setValue(0)
        self.m2_speed_spinbox.valueChanged.connect(self._on_m2_speed_spinbox_changed)
        self.m2_speed_spinbox.setEnabled(False)
        m2_speed_layout.addWidget(self.m2_speed_spinbox)
        
        m2_layout.addLayout(m2_speed_layout)
        
        # Direction buttons
        m2_dir_layout = QHBoxLayout()
        self.m2_forward_btn = QPushButton("Forward")
        self.m2_forward_btn.clicked.connect(lambda: self._set_m2_speed(50))
        self.m2_forward_btn.setEnabled(False)
        m2_dir_layout.addWidget(self.m2_forward_btn)
        
        self.m2_stop_btn = QPushButton("Stop")
        self.m2_stop_btn.clicked.connect(lambda: self._set_m2_speed(0))
        self.m2_stop_btn.setEnabled(False)
        m2_dir_layout.addWidget(self.m2_stop_btn)
        
        self.m2_backward_btn = QPushButton("Backward")
        self.m2_backward_btn.clicked.connect(lambda: self._set_m2_speed(-50))
        self.m2_backward_btn.setEnabled(False)
        m2_dir_layout.addWidget(self.m2_backward_btn)
        
        m2_layout.addLayout(m2_dir_layout)
        
        # Motor 2 status
        m2_status_layout = QHBoxLayout()
        m2_status_layout.addWidget(QLabel("Encoder:"))
        self.m2_encoder_label = QLabel("0")
        m2_status_layout.addWidget(self.m2_encoder_label)
        
        m2_status_layout.addWidget(QLabel("Speed:"))
        self.m2_speed_label = QLabel("0")
        m2_status_layout.addWidget(self.m2_speed_label)
        
        m2_status_layout.addWidget(QLabel("Current:"))
        self.m2_current_label = QLabel("0.0A")
        m2_status_layout.addWidget(self.m2_current_label)
        
        m2_status_layout.addStretch()
        m2_layout.addLayout(m2_status_layout)
        
        layout.addWidget(m2_group)
        
        # Mixed mode control
        mixed_group = QGroupBox("Mixed Mode Control")
        mixed_layout = QVBoxLayout(mixed_group)
        
        # Tank drive controls
        tank_layout = QHBoxLayout()
        
        self.tank_forward_btn = QPushButton("Forward")
        self.tank_forward_btn.clicked.connect(lambda: self._set_tank_drive(50, 50))
        self.tank_forward_btn.setEnabled(False)
        tank_layout.addWidget(self.tank_forward_btn)
        
        self.tank_backward_btn = QPushButton("Backward")
        self.tank_backward_btn.clicked.connect(lambda: self._set_tank_drive(-50, -50))
        self.tank_backward_btn.setEnabled(False)
        tank_layout.addWidget(self.tank_backward_btn)
        
        self.tank_left_btn = QPushButton("Turn Left")
        self.tank_left_btn.clicked.connect(lambda: self._set_tank_drive(-30, 30))
        self.tank_left_btn.setEnabled(False)
        tank_layout.addWidget(self.tank_left_btn)
        
        self.tank_right_btn = QPushButton("Turn Right")
        self.tank_right_btn.clicked.connect(lambda: self._set_tank_drive(30, -30))
        self.tank_right_btn.setEnabled(False)
        tank_layout.addWidget(self.tank_right_btn)
        
        self.tank_stop_btn = QPushButton("Stop All")
        self.tank_stop_btn.clicked.connect(lambda: self._set_tank_drive(0, 0))
        self.tank_stop_btn.setEnabled(False)
        tank_layout.addWidget(self.tank_stop_btn)
        
        mixed_layout.addLayout(tank_layout)
        
        layout.addWidget(mixed_group)
        
        layout.addStretch()
    
    def _setup_timers(self):
        """Setup update timers"""
        # Status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(100)  # Update every 100ms
    
    def set_roboclaw(self, roboclaw: Optional[RoboClawLinux]):
        """Set RoboClaw device instance"""
        self.roboclaw = roboclaw
        
        # Enable/disable controls based on connection
        connected = roboclaw is not None
        self.enable_checkbox.setEnabled(connected)
        
        if not connected:
            self.enable_checkbox.setChecked(False)
            self._on_motors_toggled()
    
    def _on_motors_toggled(self):
        """Handle motors enable/disable toggle"""
        self.motors_enabled = self.enable_checkbox.isChecked()
        
        # Enable/disable all motor controls
        controls = [
            self.m1_speed_slider, self.m1_speed_spinbox,
            self.m1_forward_btn, self.m1_stop_btn, self.m1_backward_btn,
            self.m2_speed_slider, self.m2_speed_spinbox,
            self.m2_forward_btn, self.m2_stop_btn, self.m2_backward_btn,
            self.tank_forward_btn, self.tank_backward_btn,
            self.tank_left_btn, self.tank_right_btn, self.tank_stop_btn
        ]
        
        for control in controls:
            control.setEnabled(self.motors_enabled and self.roboclaw is not None)
        
        # Stop motors when disabled
        if not self.motors_enabled and self.roboclaw:
            self.roboclaw.stop_motors()
            self.motor1_speed = 0
            self.motor2_speed = 0
            self.m1_speed_slider.setValue(0)
            self.m1_speed_spinbox.setValue(0)
            self.m2_speed_slider.setValue(0)
            self.m2_speed_spinbox.setValue(0)
        
        logger.info(f"Motors {'enabled' if self.motors_enabled else 'disabled'}")
    
    def _on_emergency_stop(self):
        """Handle emergency stop"""
        if self.roboclaw:
            self.roboclaw.stop_motors()
            self.motor1_speed = 0
            self.motor2_speed = 0
            self.m1_speed_slider.setValue(0)
            self.m1_speed_spinbox.setValue(0)
            self.m2_speed_slider.setValue(0)
            self.m2_speed_spinbox.setValue(0)
            
            # Disable motors
            self.enable_checkbox.setChecked(False)
            
            logger.warning("Emergency stop activated")
    
    def _on_m1_speed_changed(self, value: int):
        """Handle M1 speed slider change"""
        self.m1_speed_spinbox.setValue(value)
        self._set_m1_speed(value)
    
    def _on_m1_speed_spinbox_changed(self, value: int):
        """Handle M1 speed spinbox change"""
        self.m1_speed_slider.setValue(value)
        self._set_m1_speed(value)
    
    def _on_m2_speed_changed(self, value: int):
        """Handle M2 speed slider change"""
        self.m2_speed_spinbox.setValue(value)
        self._set_m2_speed(value)
    
    def _on_m2_speed_spinbox_changed(self, value: int):
        """Handle M2 speed spinbox change"""
        self.m2_speed_slider.setValue(value)
        self._set_m2_speed(value)
    
    def _set_m1_speed(self, speed: int):
        """Set Motor 1 speed"""
        if not self.motors_enabled or not self.roboclaw:
            return
        
        self.motor1_speed = speed
        
        try:
            if speed >= 0:
                self.roboclaw.forward_m1(speed)
            else:
                self.roboclaw.backward_m1(abs(speed))
            
            # Update UI if not already set
            if self.m1_speed_slider.value() != speed:
                self.m1_speed_slider.setValue(speed)
            if self.m1_speed_spinbox.value() != speed:
                self.m1_speed_spinbox.setValue(speed)
                
        except Exception as e:
            logger.error(f"Failed to set M1 speed: {e}")
    
    def _set_m2_speed(self, speed: int):
        """Set Motor 2 speed"""
        if not self.motors_enabled or not self.roboclaw:
            return
        
        self.motor2_speed = speed
        
        try:
            if speed >= 0:
                self.roboclaw.forward_m2(speed)
            else:
                self.roboclaw.backward_m2(abs(speed))
            
            # Update UI if not already set
            if self.m2_speed_slider.value() != speed:
                self.m2_speed_slider.setValue(speed)
            if self.m2_speed_spinbox.value() != speed:
                self.m2_speed_spinbox.setValue(speed)
                
        except Exception as e:
            logger.error(f"Failed to set M2 speed: {e}")
    
    def _set_tank_drive(self, speed1: int, speed2: int):
        """Set tank drive speeds for both motors"""
        self._set_m1_speed(speed1)
        self._set_m2_speed(speed2)
    
    def _update_status(self):
        """Update motor status displays"""
        if not self.roboclaw:
            return
        
        try:
            # Update M1 status
            enc_result = self.roboclaw.get_encoder_m1()
            if enc_result:
                encoder, status = enc_result
                self.m1_encoder_label.setText(str(encoder))
            
            speed_result = self.roboclaw.get_speed_m1()
            if speed_result:
                speed, status = speed_result
                self.m1_speed_label.setText(str(speed))
            
            # Update M2 status
            enc_result = self.roboclaw.get_encoder_m2()
            if enc_result:
                encoder, status = enc_result
                self.m2_encoder_label.setText(str(encoder))
            
            speed_result = self.roboclaw.get_speed_m2()
            if speed_result:
                speed, status = speed_result
                self.m2_speed_label.setText(str(speed))
            
            # Update currents
            currents = self.roboclaw.get_currents()
            if currents:
                current1, current2 = currents
                self.m1_current_label.setText(f"{current1:.2f}A")
                self.m2_current_label.setText(f"{current2:.2f}A")
                
        except Exception as e:
            # Don't log every failed status update to avoid spam
            pass
