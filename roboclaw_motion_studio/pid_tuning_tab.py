"""
PID tuning tab for RoboClaw Motion Studio with auto-tuning capabilities
"""

import logging
import time
import math
from typing import Optional, Tuple, List
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                            QDoubleSpinBox, QPushButton, QLabel, QSpinBox,
                            QProgressBar, QTextEdit, QTabWidget, QComboBox,
                            QCheckBox, QMessageBox, QSlider)
from PyQt6.QtCore import QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont
import numpy as np
from scipy import signal

from .roboclaw_linux import RoboClawLinux

logger = logging.getLogger(__name__)

class AutoTuneWorker(QThread):
    """Worker thread for PID auto-tuning"""
    
    progress_update = pyqtSignal(int, str)  # progress, message
    tuning_complete = pyqtSignal(bool, dict)  # success, results
    
    def __init__(self, roboclaw: RoboClawLinux, motor: int, method: str):
        super().__init__()
        self.roboclaw = roboclaw
        self.motor = motor  # 1 or 2
        self.method = method
        self.stop_tuning = False
    
    def run(self):
        """Run auto-tuning process"""
        try:
            if self.method == "step_response":
                results = self._step_response_tuning()
            elif self.method == "ziegler_nichols":
                results = self._ziegler_nichols_tuning()
            elif self.method == "relay_feedback":
                results = self._relay_feedback_tuning()
            else:
                self.tuning_complete.emit(False, {"error": "Unknown tuning method"})
                return
            
            if results and not self.stop_tuning:
                self.tuning_complete.emit(True, results)
            else:
                self.tuning_complete.emit(False, {"error": "Tuning was stopped or failed"})
                
        except Exception as e:
            logger.error(f"Auto-tuning error: {e}")
            self.tuning_complete.emit(False, {"error": str(e)})
    
    def stop(self):
        """Stop tuning process"""
        self.stop_tuning = True
    
    def _step_response_tuning(self) -> dict:
        """Perform step response auto-tuning"""
        self.progress_update.emit(10, "Starting step response test...")
        
        # Reset system
        self.roboclaw.stop_motors()
        time.sleep(0.5)
        
        # Record initial position
        initial_enc = self._get_encoder_value()
        if initial_enc is None:
            raise Exception("Cannot read encoder")
        
        self.progress_update.emit(20, "Recording baseline...")
        
        # Apply step input
        step_speed = 50  # Moderate test speed
        response_data = []
        time_data = []
        
        start_time = time.time()
        
        # Apply step input
        self._set_motor_speed(step_speed)
        self.progress_update.emit(30, "Applying step input...")
        
        # Record response for 3 seconds
        while time.time() - start_time < 3.0 and not self.stop_tuning:
            current_time = time.time() - start_time
            encoder_value = self._get_encoder_value()
            
            if encoder_value is not None:
                response_data.append(encoder_value - initial_enc)
                time_data.append(current_time)
            
            time.sleep(0.01)  # 100Hz sampling
            
            progress = 30 + int((current_time / 3.0) * 50)
            self.progress_update.emit(progress, f"Recording response... {current_time:.1f}s")
        
        # Stop motor
        self.roboclaw.stop_motors()
        self.progress_update.emit(80, "Analyzing response...")
        
        if len(response_data) < 50:
            raise Exception("Insufficient data collected")
        
        # Analyze step response
        pid_params = self._analyze_step_response(time_data, response_data)
        
        self.progress_update.emit(100, "Step response tuning complete")
        return pid_params
    
    def _relay_feedback_tuning(self) -> dict:
        """Perform relay feedback auto-tuning (similar to Ziegler-Nichols)"""
        self.progress_update.emit(10, "Starting relay feedback test...")
        
        # Reset system
        self.roboclaw.stop_motors()
        time.sleep(0.5)
        
        # Parameters for relay feedback
        relay_amplitude = 50  # PWM amplitude
        switch_threshold = 100  # Encoder count threshold for switching
        
        # Data collection
        time_data = []
        encoder_data = []
        speed_data = []
        
        start_time = time.time()
        current_direction = 1  # 1 for positive, -1 for negative
        last_encoder = self._get_encoder_value() or 0
        peaks = []
        valleys = []
        
        self.progress_update.emit(20, "Running relay feedback...")
        
        # Run relay feedback for up to 10 seconds or until we get enough oscillations
        oscillation_count = 0
        target_oscillations = 4
        
        while (time.time() - start_time < 10.0 and 
               oscillation_count < target_oscillations and 
               not self.stop_tuning):
            
            current_time = time.time() - start_time
            encoder_value = self._get_encoder_value()
            
            if encoder_value is None:
                continue
            
            # Check for direction change
            encoder_diff = encoder_value - last_encoder
            
            if abs(encoder_diff) > switch_threshold:
                current_direction *= -1
                oscillation_count += 1
                
                if current_direction > 0:
                    valleys.append((current_time, encoder_value))
                else:
                    peaks.append((current_time, encoder_value))
            
            # Apply relay feedback
            motor_speed = current_direction * relay_amplitude
            self._set_motor_speed(motor_speed)
            
            # Record data
            time_data.append(current_time)
            encoder_data.append(encoder_value)
            speed_data.append(motor_speed)
            last_encoder = encoder_value
            
            time.sleep(0.01)
            
            progress = 20 + int((current_time / 10.0) * 60)
            self.progress_update.emit(progress, f"Oscillations: {oscillation_count}/{target_oscillations}")
        
        # Stop motor
        self.roboclaw.stop_motors()
        self.progress_update.emit(80, "Analyzing oscillations...")
        
        if oscillation_count < 2:
            raise Exception("Insufficient oscillations detected")
        
        # Calculate ultimate gain and period
        pid_params = self._analyze_relay_feedback(time_data, encoder_data, peaks, valleys)
        
        self.progress_update.emit(100, "Relay feedback tuning complete")
        return pid_params
    
    def _ziegler_nichols_tuning(self) -> dict:
        """Simplified Ziegler-Nichols-like tuning"""
        # This is a simplified version - in practice, true Z-N requires finding
        # the ultimate gain that causes sustained oscillations
        return self._relay_feedback_tuning()
    
    def _analyze_step_response(self, time_data: List[float], response_data: List[int]) -> dict:
        """Analyze step response and calculate PID parameters"""
        if len(response_data) < 10:
            raise Exception("Insufficient response data")
        
        # Convert to numpy arrays
        t = np.array(time_data)
        y = np.array(response_data, dtype=float)
        
        # Find steady-state value (last 20% of data)
        steady_start = int(len(y) * 0.8)
        steady_state = np.mean(y[steady_start:])
        
        if steady_state < 10:  # Minimum movement required
            raise Exception("No significant response detected")
        
        # Normalize response (0 to 1)
        y_norm = y / steady_state
        
        # Find key characteristics
        # Rise time (10% to 90%)
        idx_10 = np.where(y_norm >= 0.1)[0]
        idx_90 = np.where(y_norm >= 0.9)[0]
        
        if len(idx_10) == 0 or len(idx_90) == 0:
            # Use simple estimates
            rise_time = t[-1] / 3  # Rough estimate
            delay_time = t[-1] / 10
        else:
            rise_time = t[idx_90[0]] - t[idx_10[0]]
            delay_time = t[idx_10[0]] if len(idx_10) > 0 else 0
        
        # Time constant (63% of steady state)
        idx_63 = np.where(y_norm >= 0.63)[0]
        time_constant = t[idx_63[0]] if len(idx_63) > 0 else rise_time
        
        # Calculate PID parameters using Cohen-Coon method
        if delay_time > 0 and time_constant > delay_time:
            K = steady_state / 50  # Process gain (response per unit input)
            L = delay_time  # Dead time
            T = time_constant - delay_time  # Time constant
            
            # Cohen-Coon tuning rules
            Kp = (1.35 / K) * (T / L)
            Ki = Kp / (2.5 * L * (1 + 0.185 * (L / T)))
            Kd = Kp * 0.37 * L / (1 + 0.185 * (L / T))
        else:
            # Fallback to simple rules
            Kp = 0.6 * (rise_time / steady_state)
            Ki = Kp / (2 * rise_time)
            Kd = Kp * rise_time / 8
        
        # Ensure reasonable bounds
        Kp = max(0.1, min(10.0, Kp))
        Ki = max(0.01, min(5.0, Ki))
        Kd = max(0.0, min(1.0, Kd))
        
        return {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "qpps": 44000,  # Default QPPS for RoboClaw
            "method": "step_response",
            "rise_time": rise_time,
            "time_constant": time_constant,
            "steady_state": steady_state
        }
    
    def _analyze_relay_feedback(self, time_data: List[float], encoder_data: List[int], 
                               peaks: List[Tuple[float, int]], valleys: List[Tuple[float, int]]) -> dict:
        """Analyze relay feedback oscillations and calculate PID parameters"""
        if len(peaks) < 2 or len(valleys) < 2:
            raise Exception("Insufficient oscillation data")
        
        # Calculate ultimate period (time between peaks)
        peak_times = [p[0] for p in peaks]
        if len(peak_times) >= 2:
            periods = [peak_times[i+1] - peak_times[i] for i in range(len(peak_times)-1)]
            ultimate_period = np.mean(periods)
        else:
            ultimate_period = 1.0  # Default
        
        # Calculate ultimate gain (this is simplified)
        # In practice, this requires iterative testing at different gains
        ultimate_gain = 2.0  # Conservative estimate
        
        # Ziegler-Nichols tuning rules for PID
        Kp = 0.6 * ultimate_gain
        Ki = Kp / (ultimate_period / 2)
        Kd = Kp * ultimate_period / 8
        
        # Ensure reasonable bounds
        Kp = max(0.1, min(10.0, Kp))
        Ki = max(0.01, min(5.0, Ki))
        Kd = max(0.0, min(1.0, Kd))
        
        return {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "qpps": 44000,
            "method": "relay_feedback",
            "ultimate_period": ultimate_period,
            "ultimate_gain": ultimate_gain,
            "oscillations": len(peaks) + len(valleys)
        }
    
    def _get_encoder_value(self) -> Optional[int]:
        """Get current encoder value for the selected motor"""
        if self.motor == 1:
            result = self.roboclaw.get_encoder_m1()
        else:
            result = self.roboclaw.get_encoder_m2()
        
        return result[0] if result else None
    
    def _set_motor_speed(self, speed: int):
        """Set motor speed for the selected motor"""
        if self.motor == 1:
            if speed >= 0:
                self.roboclaw.forward_m1(speed)
            else:
                self.roboclaw.backward_m1(abs(speed))
        else:
            if speed >= 0:
                self.roboclaw.forward_m2(speed)
            else:
                self.roboclaw.backward_m2(abs(speed))

class PIDTuningTab(QWidget):
    """PID tuning interface with auto-tuning capabilities"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.roboclaw: Optional[RoboClawLinux] = None
        self.autotune_worker: Optional[AutoTuneWorker] = None
        
        self._create_ui()
        self._setup_timers()
    
    def _create_ui(self):
        """Create user interface"""
        layout = QVBoxLayout(self)
        
        # Create tab widget for M1 and M2
        self.tab_widget = QTabWidget()
        
        # Motor 1 tab
        self.m1_tab = self._create_motor_tab(1)
        self.tab_widget.addTab(self.m1_tab, "Motor 1 PID")
        
        # Motor 2 tab
        self.m2_tab = self._create_motor_tab(2)
        self.tab_widget.addTab(self.m2_tab, "Motor 2 PID")
        
        layout.addWidget(self.tab_widget)
        
        # Auto-tuning controls
        autotune_group = QGroupBox("Auto-Tuning")
        autotune_layout = QVBoxLayout(autotune_group)
        
        # Method selection
        method_layout = QHBoxLayout()
        method_layout.addWidget(QLabel("Tuning Method:"))
        self.method_combo = QComboBox()
        self.method_combo.addItems([
            "Step Response",
            "Relay Feedback", 
            "Ziegler-Nichols"
        ])
        method_layout.addWidget(self.method_combo)
        method_layout.addStretch()
        autotune_layout.addLayout(method_layout)
        
        # Motor selection for auto-tuning
        motor_layout = QHBoxLayout()
        motor_layout.addWidget(QLabel("Motor:"))
        self.autotune_motor_combo = QComboBox()
        self.autotune_motor_combo.addItems(["Motor 1", "Motor 2"])
        motor_layout.addWidget(self.autotune_motor_combo)
        motor_layout.addStretch()
        autotune_layout.addLayout(motor_layout)
        
        # Auto-tune buttons
        button_layout = QHBoxLayout()
        self.start_autotune_btn = QPushButton("Start Auto-Tuning")
        self.start_autotune_btn.clicked.connect(self.start_autotuning)
        self.start_autotune_btn.setEnabled(False)
        button_layout.addWidget(self.start_autotune_btn)
        
        self.stop_autotune_btn = QPushButton("Stop Auto-Tuning")
        self.stop_autotune_btn.clicked.connect(self.stop_autotuning)
        self.stop_autotune_btn.setEnabled(False)
        button_layout.addWidget(self.stop_autotune_btn)
        
        button_layout.addStretch()
        autotune_layout.addLayout(button_layout)
        
        # Progress bar
        self.autotune_progress = QProgressBar()
        self.autotune_progress.setVisible(False)
        autotune_layout.addWidget(self.autotune_progress)
        
        # Results display
        self.results_text = QTextEdit()
        self.results_text.setMaximumHeight(150)
        self.results_text.setReadOnly(True)
        font = QFont("Courier")
        font.setPointSize(10)
        self.results_text.setFont(font)
        autotune_layout.addWidget(self.results_text)
        
        layout.addWidget(autotune_group)
    
    def _create_motor_tab(self, motor_num: int) -> QWidget:
        """Create PID tuning tab for a specific motor"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Current PID values group
        current_group = QGroupBox(f"Current PID Values - Motor {motor_num}")
        current_layout = QVBoxLayout(current_group)
        
        # Read current values button
        read_btn = QPushButton("Read Current Values")
        read_btn.clicked.connect(lambda: self._read_current_pid(motor_num))
        current_layout.addWidget(read_btn)
        
        # Current values display
        current_values_layout = QHBoxLayout()
        
        current_values_layout.addWidget(QLabel("Kp:"))
        current_kp_label = QLabel("0.000")
        current_kp_label.setObjectName(f"current_kp_m{motor_num}")
        current_values_layout.addWidget(current_kp_label)
        
        current_values_layout.addWidget(QLabel("Ki:"))
        current_ki_label = QLabel("0.000")
        current_ki_label.setObjectName(f"current_ki_m{motor_num}")
        current_values_layout.addWidget(current_ki_label)
        
        current_values_layout.addWidget(QLabel("Kd:"))
        current_kd_label = QLabel("0.000")
        current_kd_label.setObjectName(f"current_kd_m{motor_num}")
        current_values_layout.addWidget(current_kd_label)
        
        current_values_layout.addWidget(QLabel("QPPS:"))
        current_qpps_label = QLabel("0")
        current_qpps_label.setObjectName(f"current_qpps_m{motor_num}")
        current_values_layout.addWidget(current_qpps_label)
        
        current_values_layout.addStretch()
        current_layout.addLayout(current_values_layout)
        
        layout.addWidget(current_group)
        
        # New PID values group
        new_group = QGroupBox(f"New PID Values - Motor {motor_num}")
        new_layout = QVBoxLayout(new_group)
        
        # Kp setting
        kp_layout = QHBoxLayout()
        kp_layout.addWidget(QLabel("Kp:"))
        kp_spinbox = QDoubleSpinBox()
        kp_spinbox.setRange(0.0, 100.0)
        kp_spinbox.setDecimals(6)
        kp_spinbox.setSingleStep(0.1)
        kp_spinbox.setObjectName(f"kp_spinbox_m{motor_num}")
        kp_layout.addWidget(kp_spinbox)
        kp_layout.addStretch()
        new_layout.addLayout(kp_layout)
        
        # Ki setting
        ki_layout = QHBoxLayout()
        ki_layout.addWidget(QLabel("Ki:"))
        ki_spinbox = QDoubleSpinBox()
        ki_spinbox.setRange(0.0, 100.0)
        ki_spinbox.setDecimals(6)
        ki_spinbox.setSingleStep(0.01)
        ki_spinbox.setObjectName(f"ki_spinbox_m{motor_num}")
        ki_layout.addWidget(ki_spinbox)
        ki_layout.addStretch()
        new_layout.addLayout(ki_layout)
        
        # Kd setting
        kd_layout = QHBoxLayout()
        kd_layout.addWidget(QLabel("Kd:"))
        kd_spinbox = QDoubleSpinBox()
        kd_spinbox.setRange(0.0, 100.0)
        kd_spinbox.setDecimals(6)
        kd_spinbox.setSingleStep(0.001)
        kd_spinbox.setObjectName(f"kd_spinbox_m{motor_num}")
        kd_layout.addWidget(kd_spinbox)
        kd_layout.addStretch()
        new_layout.addLayout(kd_layout)
        
        # QPPS setting
        qpps_layout = QHBoxLayout()
        qpps_layout.addWidget(QLabel("QPPS:"))
        qpps_spinbox = QSpinBox()
        qpps_spinbox.setRange(0, 1000000)
        qpps_spinbox.setValue(44000)  # Default for RoboClaw
        qpps_spinbox.setObjectName(f"qpps_spinbox_m{motor_num}")
        qpps_layout.addWidget(qpps_spinbox)
        qpps_layout.addStretch()
        new_layout.addLayout(qpps_layout)
        
        # Write button
        write_btn = QPushButton("Write PID Values")
        write_btn.clicked.connect(lambda: self._write_pid_values(motor_num))
        new_layout.addWidget(write_btn)
        
        layout.addWidget(new_group)
        
        layout.addStretch()
        return tab
    
    def _setup_timers(self):
        """Setup update timers"""
        pass  # No regular updates needed for PID tab
    
    def set_roboclaw(self, roboclaw: Optional[RoboClawLinux]):
        """Set RoboClaw device instance"""
        self.roboclaw = roboclaw
        
        # Enable/disable controls based on connection
        connected = roboclaw is not None
        self.start_autotune_btn.setEnabled(connected)
        
        # Read current PID values if connected
        if connected:
            self._read_current_pid(1)
            self._read_current_pid(2)
    
    def _read_current_pid(self, motor_num: int):
        """Read current PID values from device"""
        if not self.roboclaw:
            return
        
        try:
            if motor_num == 1:
                result = self.roboclaw.read_velocity_pid_m1()
            else:
                result = self.roboclaw.read_velocity_pid_m2()
            
            if result:
                kp, ki, kd, qpps = result
                
                # Update current values display
                self.findChild(QLabel, f"current_kp_m{motor_num}").setText(f"{kp:.6f}")
                self.findChild(QLabel, f"current_ki_m{motor_num}").setText(f"{ki:.6f}")
                self.findChild(QLabel, f"current_kd_m{motor_num}").setText(f"{kd:.6f}")
                self.findChild(QLabel, f"current_qpps_m{motor_num}").setText(str(qpps))
                
                # Update spinboxes with current values
                self.findChild(QDoubleSpinBox, f"kp_spinbox_m{motor_num}").setValue(kp)
                self.findChild(QDoubleSpinBox, f"ki_spinbox_m{motor_num}").setValue(ki)
                self.findChild(QDoubleSpinBox, f"kd_spinbox_m{motor_num}").setValue(kd)
                self.findChild(QSpinBox, f"qpps_spinbox_m{motor_num}").setValue(qpps)
                
                logger.info(f"Read M{motor_num} PID: Kp={kp:.6f}, Ki={ki:.6f}, Kd={kd:.6f}, QPPS={qpps}")
            else:
                QMessageBox.warning(self, "Warning", f"Failed to read PID values for Motor {motor_num}")
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error reading PID values: {e}")
            logger.error(f"Error reading M{motor_num} PID values: {e}")
    
    def _write_pid_values(self, motor_num: int):
        """Write PID values to device"""
        if not self.roboclaw:
            return
        
        try:
            # Get values from spinboxes
            kp = self.findChild(QDoubleSpinBox, f"kp_spinbox_m{motor_num}").value()
            ki = self.findChild(QDoubleSpinBox, f"ki_spinbox_m{motor_num}").value()
            kd = self.findChild(QDoubleSpinBox, f"kd_spinbox_m{motor_num}").value()
            qpps = self.findChild(QSpinBox, f"qpps_spinbox_m{motor_num}").value()
            
            # Write to device
            if motor_num == 1:
                success = self.roboclaw.set_velocity_pid_m1(kp, ki, kd, qpps)
            else:
                success = self.roboclaw.set_velocity_pid_m2(kp, ki, kd, qpps)
            
            if success:
                QMessageBox.information(self, "Success", f"PID values written to Motor {motor_num}")
                logger.info(f"Wrote M{motor_num} PID: Kp={kp:.6f}, Ki={ki:.6f}, Kd={kd:.6f}, QPPS={qpps}")
                
                # Re-read to confirm
                self._read_current_pid(motor_num)
            else:
                QMessageBox.warning(self, "Warning", f"Failed to write PID values to Motor {motor_num}")
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error writing PID values: {e}")
            logger.error(f"Error writing M{motor_num} PID values: {e}")
    
    def start_autotuning(self):
        """Start PID auto-tuning process"""
        if not self.roboclaw:
            QMessageBox.warning(self, "Warning", "Please connect to a device first")
            return
        
        # Get selected motor and method
        motor_num = self.autotune_motor_combo.currentIndex() + 1
        method_text = self.method_combo.currentText()
        method = method_text.lower().replace(" ", "_").replace("-", "_")
        
        # Confirm with user
        reply = QMessageBox.question(
            self, "Start Auto-Tuning",
            f"Start auto-tuning for Motor {motor_num} using {method_text} method?\n\n"
            "This will move the motor for testing. Ensure the motor can move freely.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        # Disable controls
        self.start_autotune_btn.setEnabled(False)
        self.stop_autotune_btn.setEnabled(True)
        self.autotune_progress.setVisible(True)
        self.autotune_progress.setValue(0)
        
        # Clear previous results
        self.results_text.clear()
        
        # Start worker thread
        self.autotune_worker = AutoTuneWorker(self.roboclaw, motor_num, method)
        self.autotune_worker.progress_update.connect(self._on_autotune_progress)
        self.autotune_worker.tuning_complete.connect(self._on_autotune_complete)
        self.autotune_worker.start()
        
        logger.info(f"Started auto-tuning for M{motor_num} using {method}")
    
    def stop_autotuning(self):
        """Stop auto-tuning process"""
        if self.autotune_worker:
            self.autotune_worker.stop()
            self.autotune_worker.wait(3000)  # Wait up to 3 seconds
        
        self._reset_autotune_ui()
        
        # Stop motors
        if self.roboclaw:
            self.roboclaw.stop_motors()
        
        logger.info("Auto-tuning stopped by user")
    
    def _on_autotune_progress(self, progress: int, message: str):
        """Handle auto-tuning progress update"""
        self.autotune_progress.setValue(progress)
        self.results_text.append(f"[{progress:3d}%] {message}")
    
    def _on_autotune_complete(self, success: bool, results: dict):
        """Handle auto-tuning completion"""
        self._reset_autotune_ui()
        
        if success:
            # Display results
            self.results_text.append("\n=== AUTO-TUNING COMPLETE ===")
            self.results_text.append(f"Method: {results.get('method', 'unknown')}")
            self.results_text.append(f"Kp: {results['Kp']:.6f}")
            self.results_text.append(f"Ki: {results['Ki']:.6f}")
            self.results_text.append(f"Kd: {results['Kd']:.6f}")
            self.results_text.append(f"QPPS: {results['qpps']}")
            
            # Add method-specific results
            if 'rise_time' in results:
                self.results_text.append(f"Rise Time: {results['rise_time']:.3f}s")
            if 'ultimate_period' in results:
                self.results_text.append(f"Ultimate Period: {results['ultimate_period']:.3f}s")
            
            # Ask if user wants to apply the values
            motor_num = self.autotune_motor_combo.currentIndex() + 1
            reply = QMessageBox.question(
                self, "Apply Results",
                f"Auto-tuning complete!\n\n"
                f"Suggested PID values for Motor {motor_num}:\n"
                f"Kp: {results['Kp']:.6f}\n"
                f"Ki: {results['Ki']:.6f}\n"
                f"Kd: {results['Kd']:.6f}\n"
                f"QPPS: {results['qpps']}\n\n"
                f"Apply these values to the motor?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.Yes:
                # Update spinboxes
                self.findChild(QDoubleSpinBox, f"kp_spinbox_m{motor_num}").setValue(results['Kp'])
                self.findChild(QDoubleSpinBox, f"ki_spinbox_m{motor_num}").setValue(results['Ki'])
                self.findChild(QDoubleSpinBox, f"kd_spinbox_m{motor_num}").setValue(results['Kd'])
                self.findChild(QSpinBox, f"qpps_spinbox_m{motor_num}").setValue(results['qpps'])
                
                # Write to device
                self._write_pid_values(motor_num)
            
            logger.info(f"Auto-tuning completed successfully: {results}")
        else:
            error_msg = results.get('error', 'Unknown error')
            self.results_text.append(f"\n=== AUTO-TUNING FAILED ===")
            self.results_text.append(f"Error: {error_msg}")
            QMessageBox.warning(self, "Auto-Tuning Failed", f"Auto-tuning failed: {error_msg}")
            logger.error(f"Auto-tuning failed: {error_msg}")
        
        # Stop motors
        if self.roboclaw:
            self.roboclaw.stop_motors()
    
    def _reset_autotune_ui(self):
        """Reset auto-tuning UI to initial state"""
        self.start_autotune_btn.setEnabled(True)
        self.stop_autotune_btn.setEnabled(False)
        self.autotune_progress.setVisible(False)
