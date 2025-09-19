"""
Real-time monitoring tab for RoboClaw Motion Studio
"""

import logging
import time
from collections import deque
from typing import Optional, Dict, List
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                            QLabel, QProgressBar, QGridLayout, QTabWidget,
                            QPushButton, QCheckBox, QSpinBox, QComboBox)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QPalette
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

from .roboclaw_protocol import RoboClawProtocol

logger = logging.getLogger(__name__)

class MonitoringTab(QWidget):
    """Real-time monitoring and data visualization tab"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.roboclaw: Optional[RoboClawProtocol] = None
        
        # Data storage for plotting
        self.max_data_points = 1000
        self.time_data = deque(maxlen=self.max_data_points)
        self.m1_encoder_data = deque(maxlen=self.max_data_points)
        self.m2_encoder_data = deque(maxlen=self.max_data_points)
        self.m1_speed_data = deque(maxlen=self.max_data_points)
        self.m2_speed_data = deque(maxlen=self.max_data_points)
        self.m1_current_data = deque(maxlen=self.max_data_points)
        self.m2_current_data = deque(maxlen=self.max_data_points)
        self.voltage_data = deque(maxlen=self.max_data_points)
        self.temperature_data = deque(maxlen=self.max_data_points)
        
        self.start_time = time.time()
        self.monitoring_enabled = False
        
        self._create_ui()
        self._setup_timers()
    
    def _create_ui(self):
        """Create user interface"""
        layout = QVBoxLayout(self)
        
        # Monitoring controls
        controls_group = QGroupBox("Monitoring Controls")
        controls_layout = QHBoxLayout(controls_group)
        
        self.enable_monitoring_cb = QCheckBox("Enable Monitoring")
        self.enable_monitoring_cb.stateChanged.connect(self._on_monitoring_toggled)
        controls_layout.addWidget(self.enable_monitoring_cb)
        
        self.clear_data_btn = QPushButton("Clear Data")
        self.clear_data_btn.clicked.connect(self._clear_data)
        controls_layout.addWidget(self.clear_data_btn)
        
        controls_layout.addWidget(QLabel("Update Rate (Hz):"))
        self.update_rate_spin = QSpinBox()
        self.update_rate_spin.setRange(1, 100)
        self.update_rate_spin.setValue(10)
        self.update_rate_spin.valueChanged.connect(self._on_update_rate_changed)
        controls_layout.addWidget(self.update_rate_spin)
        
        controls_layout.addStretch()
        layout.addWidget(controls_group)
        
        # Create tab widget for different monitoring views
        self.tab_widget = QTabWidget()
        
        # Real-time status tab
        self.status_tab = self._create_status_tab()
        self.tab_widget.addTab(self.status_tab, "Status")
        
        # Data plots tab
        self.plots_tab = self._create_plots_tab()
        self.tab_widget.addTab(self.plots_tab, "Data Plots")
        
        layout.addWidget(self.tab_widget)
    
    def _create_status_tab(self) -> QWidget:
        """Create real-time status display tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Motor status group
        motor_group = QGroupBox("Motor Status")
        motor_layout = QGridLayout(motor_group)
        
        # Headers
        motor_layout.addWidget(QLabel("<b>Parameter</b>"), 0, 0)
        motor_layout.addWidget(QLabel("<b>Motor 1</b>"), 0, 1)
        motor_layout.addWidget(QLabel("<b>Motor 2</b>"), 0, 2)
        motor_layout.addWidget(QLabel("<b>Units</b>"), 0, 3)
        
        # Encoder position
        motor_layout.addWidget(QLabel("Encoder Position:"), 1, 0)
        self.m1_encoder_status = QLabel("0")
        self.m1_encoder_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m1_encoder_status, 1, 1)
        self.m2_encoder_status = QLabel("0")
        self.m2_encoder_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m2_encoder_status, 1, 2)
        motor_layout.addWidget(QLabel("counts"), 1, 3)
        
        # Speed
        motor_layout.addWidget(QLabel("Speed:"), 2, 0)
        self.m1_speed_status = QLabel("0")
        self.m1_speed_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m1_speed_status, 2, 1)
        self.m2_speed_status = QLabel("0")
        self.m2_speed_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m2_speed_status, 2, 2)
        motor_layout.addWidget(QLabel("QPPS"), 2, 3)
        
        # Current
        motor_layout.addWidget(QLabel("Current:"), 3, 0)
        self.m1_current_status = QLabel("0.00")
        self.m1_current_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m1_current_status, 3, 1)
        self.m2_current_status = QLabel("0.00")
        self.m2_current_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m2_current_status, 3, 2)
        motor_layout.addWidget(QLabel("Amps"), 3, 3)
        
        # PWM
        motor_layout.addWidget(QLabel("PWM:"), 4, 0)
        self.m1_pwm_status = QLabel("0")
        self.m1_pwm_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m1_pwm_status, 4, 1)
        self.m2_pwm_status = QLabel("0")
        self.m2_pwm_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        motor_layout.addWidget(self.m2_pwm_status, 4, 2)
        motor_layout.addWidget(QLabel("duty"), 4, 3)
        
        layout.addWidget(motor_group)
        
        # System status group
        system_group = QGroupBox("System Status")
        system_layout = QGridLayout(system_group)
        
        # Main battery voltage
        system_layout.addWidget(QLabel("Main Battery:"), 0, 0)
        self.main_voltage_status = QLabel("0.00")
        self.main_voltage_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        system_layout.addWidget(self.main_voltage_status, 0, 1)
        system_layout.addWidget(QLabel("V"), 0, 2)
        
        # Battery level bar
        self.battery_progress = QProgressBar()
        self.battery_progress.setRange(0, 100)
        self.battery_progress.setValue(0)
        system_layout.addWidget(self.battery_progress, 0, 3, 1, 2)
        
        # Logic battery voltage
        system_layout.addWidget(QLabel("Logic Battery:"), 1, 0)
        self.logic_voltage_status = QLabel("0.00")
        self.logic_voltage_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        system_layout.addWidget(self.logic_voltage_status, 1, 1)
        system_layout.addWidget(QLabel("V"), 1, 2)
        
        # Temperature
        system_layout.addWidget(QLabel("Temperature:"), 2, 0)
        self.temperature_status = QLabel("0.0")
        self.temperature_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        system_layout.addWidget(self.temperature_status, 2, 1)
        system_layout.addWidget(QLabel("°C"), 2, 2)
        
        # Temperature bar
        self.temp_progress = QProgressBar()
        self.temp_progress.setRange(0, 100)
        self.temp_progress.setValue(0)
        system_layout.addWidget(self.temp_progress, 2, 3, 1, 2)
        
        # Error status
        system_layout.addWidget(QLabel("Error Status:"), 3, 0)
        self.error_status = QLabel("No errors")
        self.error_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        system_layout.addWidget(self.error_status, 3, 1, 1, 4)
        
        layout.addWidget(system_group)
        
        # Data logging controls
        logging_group = QGroupBox("Data Logging")
        logging_layout = QHBoxLayout(logging_group)
        
        self.log_to_file_cb = QCheckBox("Log to File")
        logging_layout.addWidget(self.log_to_file_cb)
        
        self.log_file_btn = QPushButton("Choose Log File...")
        self.log_file_btn.setEnabled(False)
        logging_layout.addWidget(self.log_file_btn)
        
        logging_layout.addWidget(QLabel("Samples Collected:"))
        self.samples_count_label = QLabel("0")
        logging_layout.addWidget(self.samples_count_label)
        
        logging_layout.addStretch()
        layout.addWidget(logging_group)
        
        layout.addStretch()
        return tab
    
    def _create_plots_tab(self) -> QWidget:
        """Create data visualization tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Plot controls
        plot_controls_group = QGroupBox("Plot Controls")
        plot_controls_layout = QHBoxLayout(plot_controls_group)
        
        plot_controls_layout.addWidget(QLabel("Plot Type:"))
        self.plot_type_combo = QComboBox()
        self.plot_type_combo.addItems([
            "Encoder Position",
            "Motor Speed",
            "Motor Current",
            "System Voltage",
            "Temperature"
        ])
        self.plot_type_combo.currentTextChanged.connect(self._update_plots)
        plot_controls_layout.addWidget(self.plot_type_combo)
        
        plot_controls_layout.addWidget(QLabel("Time Range (s):"))
        self.time_range_spin = QSpinBox()
        self.time_range_spin.setRange(10, 300)
        self.time_range_spin.setValue(60)
        self.time_range_spin.valueChanged.connect(self._update_plots)
        plot_controls_layout.addWidget(self.time_range_spin)
        
        self.auto_scale_cb = QCheckBox("Auto Scale Y-axis")
        self.auto_scale_cb.setChecked(True)
        self.auto_scale_cb.stateChanged.connect(self._update_plots)
        plot_controls_layout.addWidget(self.auto_scale_cb)
        
        plot_controls_layout.addStretch()
        layout.addWidget(plot_controls_group)
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(12, 8))
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        
        # Create subplots
        self.ax1 = self.figure.add_subplot(2, 1, 1)
        self.ax2 = self.figure.add_subplot(2, 1, 2)
        
        # Initialize plots
        self._init_plots()
        
        return tab
    
    def _init_plots(self):
        """Initialize plot displays"""
        self.ax1.set_title("Motor 1 Data")
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_xlabel("Time (s)")
        
        self.ax2.set_title("Motor 2 Data")
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_xlabel("Time (s)")
        
        self.figure.tight_layout()
        self.canvas.draw()
    
    def _setup_timers(self):
        """Setup update timers"""
        # Data collection timer
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self._collect_data)
        
        # Plot update timer (slower than data collection)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(1000)  # Update plots every second
    
    def set_roboclaw(self, roboclaw: Optional[RoboClawProtocol]):
        """Set RoboClaw device instance"""
        self.roboclaw = roboclaw
        
        # Enable/disable controls based on connection
        connected = roboclaw is not None
        self.enable_monitoring_cb.setEnabled(connected)
        
        if not connected:
            self.enable_monitoring_cb.setChecked(False)
            self._on_monitoring_toggled()
    
    def _on_monitoring_toggled(self):
        """Handle monitoring enable/disable"""
        self.monitoring_enabled = self.enable_monitoring_cb.isChecked()
        
        if self.monitoring_enabled:
            # Start data collection
            update_rate = self.update_rate_spin.value()
            interval = int(1000 / update_rate)  # Convert Hz to ms
            self.data_timer.start(interval)
            self.start_time = time.time()
            logger.info(f"Monitoring started at {update_rate} Hz")
        else:
            # Stop data collection
            self.data_timer.stop()
            logger.info("Monitoring stopped")
    
    def _on_update_rate_changed(self):
        """Handle update rate change"""
        if self.monitoring_enabled:
            update_rate = self.update_rate_spin.value()
            interval = int(1000 / update_rate)
            self.data_timer.start(interval)  # Restart with new interval
            logger.info(f"Monitoring rate changed to {update_rate} Hz")
    
    def _clear_data(self):
        """Clear all collected data"""
        self.time_data.clear()
        self.m1_encoder_data.clear()
        self.m2_encoder_data.clear()
        self.m1_speed_data.clear()
        self.m2_speed_data.clear()
        self.m1_current_data.clear()
        self.m2_current_data.clear()
        self.voltage_data.clear()
        self.temperature_data.clear()
        
        self.start_time = time.time()
        self.samples_count_label.setText("0")
        
        self._update_plots()
        logger.info("Monitoring data cleared")
    
    def _collect_data(self):
        """Collect data from RoboClaw device"""
        if not self.roboclaw or not self.monitoring_enabled:
            return
        
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        try:
            # Read encoder positions
            m1_enc_result = self.roboclaw.get_encoder_m1()
            m2_enc_result = self.roboclaw.get_encoder_m2()
            
            m1_encoder = m1_enc_result[0] if m1_enc_result else 0
            m2_encoder = m2_enc_result[0] if m2_enc_result else 0
            
            self.m1_encoder_data.append(m1_encoder)
            self.m2_encoder_data.append(m2_encoder)
            
            # Read speeds
            m1_speed_result = self.roboclaw.get_speed_m1()
            m2_speed_result = self.roboclaw.get_speed_m2()
            
            m1_speed = m1_speed_result[0] if m1_speed_result else 0
            m2_speed = m2_speed_result[0] if m2_speed_result else 0
            
            self.m1_speed_data.append(m1_speed)
            self.m2_speed_data.append(m2_speed)
            
            # Read currents
            currents = self.roboclaw.get_currents()
            if currents:
                m1_current, m2_current = currents
            else:
                m1_current, m2_current = 0.0, 0.0
            
            self.m1_current_data.append(m1_current)
            self.m2_current_data.append(m2_current)
            
            # Read system parameters
            main_voltage = self.roboclaw.get_main_battery_voltage() or 0.0
            self.voltage_data.append(main_voltage)
            
            temperature = self.roboclaw.get_temperature() or 0.0
            self.temperature_data.append(temperature)
            
            # Update status displays
            self._update_status_displays(
                m1_encoder, m2_encoder, m1_speed, m2_speed,
                m1_current, m2_current, main_voltage, temperature
            )
            
            # Update sample count
            self.samples_count_label.setText(str(len(self.time_data)))
            
        except Exception as e:
            logger.error(f"Data collection error: {e}")
    
    def _update_status_displays(self, m1_encoder, m2_encoder, m1_speed, m2_speed,
                               m1_current, m2_current, main_voltage, temperature):
        """Update real-time status displays"""
        
        # Motor status
        self.m1_encoder_status.setText(f"{m1_encoder:,}")
        self.m2_encoder_status.setText(f"{m2_encoder:,}")
        self.m1_speed_status.setText(f"{m1_speed:,}")
        self.m2_speed_status.setText(f"{m2_speed:,}")
        self.m1_current_status.setText(f"{m1_current:.2f}")
        self.m2_current_status.setText(f"{m2_current:.2f}")
        
        # PWM values
        pwm_values = self.roboclaw.get_pwm_values()
        if pwm_values:
            m1_pwm, m2_pwm = pwm_values
            self.m1_pwm_status.setText(f"{m1_pwm}")
            self.m2_pwm_status.setText(f"{m2_pwm}")
        
        # System status
        self.main_voltage_status.setText(f"{main_voltage:.2f}")
        self.temperature_status.setText(f"{temperature:.1f}")
        
        # Logic voltage
        logic_voltage = self.roboclaw.get_logic_battery_voltage()
        if logic_voltage:
            self.logic_voltage_status.setText(f"{logic_voltage:.2f}")
        
        # Battery level (assuming 12V nominal)
        battery_percent = min(100, max(0, (main_voltage - 10.5) / (12.6 - 10.5) * 100))
        self.battery_progress.setValue(int(battery_percent))
        
        # Temperature level (0-85°C range)
        temp_percent = min(100, max(0, temperature / 85.0 * 100))
        self.temp_progress.setValue(int(temp_percent))
        
        # Set temperature color based on level
        if temperature > 70:
            self.temp_progress.setStyleSheet("QProgressBar::chunk { background-color: red; }")
        elif temperature > 50:
            self.temp_progress.setStyleSheet("QProgressBar::chunk { background-color: orange; }")
        else:
            self.temp_progress.setStyleSheet("QProgressBar::chunk { background-color: green; }")
        
        # Error status
        error_status = self.roboclaw.get_error_status()
        if error_status is not None:
            if error_status == 0:
                self.error_status.setText("No errors")
                self.error_status.setStyleSheet("color: green;")
            else:
                self.error_status.setText(f"Error: 0x{error_status:08X}")
                self.error_status.setStyleSheet("color: red;")
    
    def _update_plots(self):
        """Update data visualization plots"""
        if len(self.time_data) < 2:
            return
        
        plot_type = self.plot_type_combo.currentText()
        time_range = self.time_range_spin.value()
        auto_scale = self.auto_scale_cb.isChecked()
        
        # Convert deques to numpy arrays for plotting
        times = np.array(list(self.time_data))
        
        # Filter data to time range
        current_time = times[-1] if len(times) > 0 else 0
        mask = times >= (current_time - time_range)
        times_filtered = times[mask]
        
        # Clear previous plots
        self.ax1.clear()
        self.ax2.clear()
        
        if plot_type == "Encoder Position":
            m1_data = np.array(list(self.m1_encoder_data))[mask]
            m2_data = np.array(list(self.m2_encoder_data))[mask]
            
            self.ax1.plot(times_filtered, m1_data, 'b-', linewidth=2, label='M1 Encoder')
            self.ax1.set_title("Motor 1 Encoder Position")
            self.ax1.set_ylabel("Counts")
            
            self.ax2.plot(times_filtered, m2_data, 'r-', linewidth=2, label='M2 Encoder')
            self.ax2.set_title("Motor 2 Encoder Position")
            self.ax2.set_ylabel("Counts")
            
        elif plot_type == "Motor Speed":
            m1_data = np.array(list(self.m1_speed_data))[mask]
            m2_data = np.array(list(self.m2_speed_data))[mask]
            
            self.ax1.plot(times_filtered, m1_data, 'b-', linewidth=2, label='M1 Speed')
            self.ax1.set_title("Motor 1 Speed")
            self.ax1.set_ylabel("QPPS")
            
            self.ax2.plot(times_filtered, m2_data, 'r-', linewidth=2, label='M2 Speed')
            self.ax2.set_title("Motor 2 Speed")
            self.ax2.set_ylabel("QPPS")
            
        elif plot_type == "Motor Current":
            m1_data = np.array(list(self.m1_current_data))[mask]
            m2_data = np.array(list(self.m2_current_data))[mask]
            
            self.ax1.plot(times_filtered, m1_data, 'b-', linewidth=2, label='M1 Current')
            self.ax1.set_title("Motor 1 Current")
            self.ax1.set_ylabel("Amps")
            
            self.ax2.plot(times_filtered, m2_data, 'r-', linewidth=2, label='M2 Current')
            self.ax2.set_title("Motor 2 Current")
            self.ax2.set_ylabel("Amps")
            
        elif plot_type == "System Voltage":
            voltage_data = np.array(list(self.voltage_data))[mask]
            
            self.ax1.plot(times_filtered, voltage_data, 'g-', linewidth=2, label='Main Battery')
            self.ax1.set_title("Main Battery Voltage")
            self.ax1.set_ylabel("Volts")
            
            # Show both voltages on same plot for comparison
            if len(times_filtered) > 0:
                self.ax2.axhline(y=12.6, color='g', linestyle='--', alpha=0.7, label='Full (12.6V)')
                self.ax2.axhline(y=12.0, color='y', linestyle='--', alpha=0.7, label='Good (12.0V)')
                self.ax2.axhline(y=11.0, color='orange', linestyle='--', alpha=0.7, label='Low (11.0V)')
                self.ax2.axhline(y=10.5, color='r', linestyle='--', alpha=0.7, label='Critical (10.5V)')
                self.ax2.plot(times_filtered, voltage_data, 'g-', linewidth=2, label='Main Battery')
                self.ax2.set_title("Battery Voltage with Thresholds")
                self.ax2.set_ylabel("Volts")
                self.ax2.legend()
            
        elif plot_type == "Temperature":
            temp_data = np.array(list(self.temperature_data))[mask]
            
            self.ax1.plot(times_filtered, temp_data, 'm-', linewidth=2, label='Temperature')
            self.ax1.set_title("Device Temperature")
            self.ax1.set_ylabel("°C")
            
            # Show temperature thresholds
            if len(times_filtered) > 0:
                self.ax2.axhline(y=85, color='r', linestyle='--', alpha=0.7, label='Max (85°C)')
                self.ax2.axhline(y=70, color='orange', linestyle='--', alpha=0.7, label='High (70°C)')
                self.ax2.axhline(y=50, color='y', linestyle='--', alpha=0.7, label='Warm (50°C)')
                self.ax2.plot(times_filtered, temp_data, 'm-', linewidth=2, label='Temperature')
                self.ax2.set_title("Temperature with Thresholds")
                self.ax2.set_ylabel("°C")
                self.ax2.legend()
        
        # Configure axes
        for ax in [self.ax1, self.ax2]:
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("Time (s)")
            
            if not auto_scale and len(times_filtered) > 0:
                ax.set_xlim(current_time - time_range, current_time)
        
        self.figure.tight_layout()
        self.canvas.draw()
    
    def update_data(self):
        """Public method to update monitoring data (called from main window)"""
        if self.monitoring_enabled:
            self._collect_data()

    def refresh_data(self):
        if not self.roboclaw:
            return
        try:
            mbatt = self.roboclaw.read_main_battery_voltage()
            lbatt = self.roboclaw.read_logic_battery_voltage()
            temp = self.roboclaw.read_temperature()
            enc1,_ = self.roboclaw.read_enc_m1()
            enc2,_ = self.roboclaw.read_enc_m2()
            cur1,cur2 = self.roboclaw.read_currents()
            pwm1,pwm2 = self.roboclaw.read_pwms()
            speed1,_ = self.roboclaw.read_speed_m1()
            speed2,_ = self.roboclaw.read_speed_m2()

            # Update status displays
            self._update_status_displays(
                enc1, enc2, speed1, speed2,
                cur1, cur2, mbatt, temp
            )
        except Exception as e:
            print(f"Monitor update error: {e}")
