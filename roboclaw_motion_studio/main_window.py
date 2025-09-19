"""
Main window for RoboClaw Motion Studio
"""

import sys
import os
import json
import logging
from typing import Optional, Dict, Any
from PyQt6.QtWidgets import (QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, 
                            QTabWidget, QMenuBar, QStatusBar, QMessageBox,
                            QFileDialog, QProgressBar, QLabel)
from PyQt6.QtCore import QTimer, QThread, pyqtSignal, QSettings
from PyQt6.QtGui import QAction, QIcon
from PyQt6.QtCore import Qt

from .device_connection_tab import DeviceConnectionTab
from .motor_control_tab import MotorControlTab
from .pid_tuning_tab import PIDTuningTab
from .monitoring_tab import MonitoringTab
from .configuration_tab import ConfigurationTab
from .roboclaw_protocol import RoboClawProtocol

import threading, queue, time
from logging.handlers import RotatingFileHandler

logger = logging.getLogger(__name__)

class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboClaw Motion Studio")
        self.setMinimumSize(1200, 800)
        
        # Application settings
        self.settings = QSettings("RoboClawTools", "MotionStudio")
        
        # RoboClaw device instance
        self.roboclaw: Optional[RoboClawProtocol] = None
        self.device_connected = False
        
        # Snapshot thread
        self._snapshot_thread = None
        self._snapshot_stop = threading.Event()
        self._snapshot_interval = 0.1  # 10 Hz
        
        # Setup logging
        self._setup_logging()
        
        # Create UI components
        self._create_menus()
        self._create_status_bar()
        self._create_central_widget()
        self._create_toolbar()
        
        # Setup timers
        self._setup_timers()
        
        # Restore window state
        self._restore_settings()
        
        logger.info("RoboClaw Motion Studio initialized")
    
    def _setup_logging(self):
        """Setup application logging"""
        log_level = logging.INFO
        if self.settings.value("debug_mode", False, type=bool):
            log_level = logging.DEBUG
        
        for h in list(logging.getLogger().handlers):
            logging.getLogger().removeHandler(h)
        log_file = 'roboclaw_motion_studio.log'
        rotating = RotatingFileHandler(log_file, maxBytes=512000, backupCount=3)
        rotating.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(name)s: %(message)s'))
        stream = logging.StreamHandler()
        stream.setFormatter(logging.Formatter('%(levelname)s %(name)s: %(message)s'))
        logging.getLogger().addHandler(rotating)
        logging.getLogger().addHandler(stream)
        logging.getLogger().setLevel(log_level)
    
    def _create_menus(self):
        """Create application menus"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("&File")
        
        # Connect action
        self.connect_action = QAction("&Connect Device", self)
        self.connect_action.setShortcut("Ctrl+C")
        self.connect_action.triggered.connect(self._on_connect_device)
        file_menu.addAction(self.connect_action)
        
        # Disconnect action
        self.disconnect_action = QAction("&Disconnect Device", self)
        self.disconnect_action.setShortcut("Ctrl+D")
        self.disconnect_action.setEnabled(False)
        self.disconnect_action.triggered.connect(self._on_disconnect_device)
        file_menu.addAction(self.disconnect_action)
        
        file_menu.addSeparator()
        
        # Load configuration
        load_config_action = QAction("&Load Configuration...", self)
        load_config_action.setShortcut("Ctrl+O")
        load_config_action.triggered.connect(self._on_load_configuration)
        file_menu.addAction(load_config_action)
        
        # Save configuration
        save_config_action = QAction("&Save Configuration...", self)
        save_config_action.setShortcut("Ctrl+S")
        save_config_action.triggered.connect(self._on_save_configuration)
        file_menu.addAction(save_config_action)
        
        file_menu.addSeparator()
        
        # Exit
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Tools menu
        tools_menu = menubar.addMenu("&Tools")
        
        # Auto-tune PID
        autotune_action = QAction("&Auto-tune PID", self)
        autotune_action.triggered.connect(self._on_autotune_pid)
        tools_menu.addAction(autotune_action)
        
        # Reset encoders
        reset_encoders_action = QAction("&Reset Encoders", self)
        reset_encoders_action.triggered.connect(self._on_reset_encoders)
        tools_menu.addAction(reset_encoders_action)
        
        # Emergency stop
        emergency_stop_action = QAction("&Emergency Stop", self)
        emergency_stop_action.setShortcut("Escape")
        emergency_stop_action.triggered.connect(self._on_emergency_stop)
        tools_menu.addAction(emergency_stop_action)
        
        # View menu
        view_menu = menubar.addMenu("&View")
        
        # Debug mode
        self.debug_action = QAction("&Debug Mode", self)
        self.debug_action.setCheckable(True)
        self.debug_action.setChecked(self.settings.value("debug_mode", False, type=bool))
        self.debug_action.triggered.connect(self._on_toggle_debug)
        view_menu.addAction(self.debug_action)
        
        # Help menu
        help_menu = menubar.addMenu("&Help")
        
        # About
        about_action = QAction("&About", self)
        about_action.triggered.connect(self._on_about)
        help_menu.addAction(about_action)
    
    def _create_status_bar(self):
        """Create status bar"""
        self.status_bar = self.statusBar()
        
        # Connection status
        self.connection_status_label = QLabel("Disconnected")
        self.status_bar.addWidget(self.connection_status_label)
        
        # Progress bar for operations
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.status_bar.addPermanentWidget(self.progress_bar)
        
        # Device info
        self.device_info_label = QLabel("")
        self.status_bar.addPermanentWidget(self.device_info_label)
    
    def _create_central_widget(self):
        """Create main tab widget"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        layout.addWidget(self.tab_widget)
        
        # Create tabs
        self.connection_tab = DeviceConnectionTab(self)
        self.motor_control_tab = MotorControlTab(self)
        self.pid_tuning_tab = PIDTuningTab(self)
        self.monitoring_tab = MonitoringTab(self)
        self.configuration_tab = ConfigurationTab(self)
        
        # Add tabs
        self.tab_widget.addTab(self.connection_tab, "Connection")
        self.tab_widget.addTab(self.motor_control_tab, "Motor Control")
        self.tab_widget.addTab(self.pid_tuning_tab, "PID Tuning")
        self.tab_widget.addTab(self.monitoring_tab, "Monitoring")
        self.tab_widget.addTab(self.configuration_tab, "Configuration")
        
        # Connect signals
        self.connection_tab.device_connected.connect(self._on_device_connected)
        self.connection_tab.device_disconnected.connect(self._on_device_disconnected)
    
    def _create_toolbar(self):
        """Create application toolbar"""
        tb = self.addToolBar('Main')
        tb.setMovable(False)
        
        # Connect action
        self.action_connect = QAction(QIcon.fromTheme('network-connect'), 'Connect', self)
        self.action_connect.triggered.connect(self._on_connect_device)
        tb.addAction(self.action_connect)
        
        # Disconnect action
        self.action_disconnect = QAction(QIcon.fromTheme('network-disconnect'), 'Disconnect', self)
        self.action_disconnect.triggered.connect(self._on_disconnect_device)
        self.action_disconnect.setEnabled(False)
        tb.addAction(self.action_disconnect)
        
        tb.addSeparator()
        
        # Monitoring actions
        self.action_monitor_start = QAction('Start Monitoring', self)
        self.action_monitor_start.triggered.connect(lambda: self._set_monitoring(True))
        tb.addAction(self.action_monitor_start)
        
        self.action_monitor_stop = QAction('Stop Monitoring', self)
        self.action_monitor_stop.triggered.connect(lambda: self._set_monitoring(False))
        tb.addAction(self.action_monitor_stop)
        
        tb.addSeparator()
        
        # Emergency stop
        self.action_emergency = QAction(QIcon.fromTheme('process-stop'), 'Emergency Stop', self)
        self.action_emergency.triggered.connect(self._on_emergency_stop)
        tb.addAction(self.action_emergency)
        
        tb.addSeparator()
        
        # Theme toggle
        self.action_theme = QAction('Toggle Theme', self)
        self.action_theme.setCheckable(True)
        self.action_theme.triggered.connect(self._toggle_theme)
        tb.addAction(self.action_theme)
    
    def _toggle_theme(self):
        """Toggle between light and dark theme"""
        dark = self.action_theme.isChecked()
        if dark:
            self.settings.setValue('theme', 'dark')
            self._apply_theme('dark')
        else:
            self.settings.setValue('theme', 'light')
            self._apply_theme('light')
    
    def _apply_theme(self, theme: str):
        """Apply the selected theme"""
        if theme == 'dark':
            self.setStyleSheet("""
                QWidget { background-color:#232629; color:#ddd; }
                QGroupBox { border:1px solid #444; margin-top:6px; }
                QGroupBox::title { subcontrol-origin: margin; left:8px; padding:0 4px; }
                QPushButton { background:#444; border:1px solid #555; padding:4px; }
                QPushButton:hover { background:#555; }
                QTabBar::tab:selected { background:#444; }
                QStatusBar { background:#1e1e1e; }
            """)
        else:
            self.setStyleSheet("")
    
    def _setup_timers(self):
        """Setup update timers"""
        # Status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(1000)  # Update every second
        
        # Monitoring update timer
        self.monitoring_timer = QTimer()
        self.monitoring_timer.timeout.connect(self._update_monitoring)
        self.monitoring_timer.start(100)  # Update every 100ms
    
    def _restore_settings(self):
        """Restore application settings"""
        # Window geometry
        geometry = self.settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)
        
        # Window state
        state = self.settings.value("windowState")
        if state:
            self.restoreState(state)
        
        # Theme
        theme = self.settings.value('theme','light')
        self.action_theme.setChecked(theme=='dark')
        self._apply_theme(theme)
    
    def _save_settings(self):
        """Save application settings"""
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        self.settings.setValue("debug_mode", self.debug_action.isChecked())
    
    def closeEvent(self, event):
        """Handle application close event"""
        self._stop_snapshot_thread()
        
        if self.device_connected:
            self._on_disconnect_device()
        
        self._save_settings()
        event.accept()
    
    def _on_connect_device(self):
        """Handle connect device action"""
        self.tab_widget.setCurrentWidget(self.connection_tab)
        self.connection_tab.connect_device()
    
    def _on_disconnect_device(self):
        """Handle disconnect device action"""
        self.connection_tab.disconnect_device()
    
    def _on_device_connected(self, roboclaw):
        """Handle device connection"""
        # Accept protocol instance
        self.roboclaw = roboclaw
        
        # Update UI
        self.connect_action.setEnabled(False)
        self.disconnect_action.setEnabled(True)
        self.action_connect.setEnabled(False)
        self.action_disconnect.setEnabled(True)
        self.connection_status_label.setText("Connected")
        
        # Get device info
        version = self.roboclaw.get_version()
        if version:
            self.device_info_label.setText(f"RoboClaw {version}")
        
        # Propagate to tabs
        self.motor_control_tab.set_roboclaw(roboclaw)
        self.pid_tuning_tab.set_roboclaw(roboclaw)
        self.monitoring_tab.set_roboclaw(roboclaw)
        self.configuration_tab.set_roboclaw(roboclaw)
        
        self._start_snapshot_thread()
        
        logger.info(f"Device connected: {version}")
    
    def _on_device_disconnected(self):
        """Handle device disconnection"""
        self.roboclaw = None
        self.device_connected = False
        
        # Update UI
        self.connect_action.setEnabled(True)
        self.disconnect_action.setEnabled(False)
        self.action_connect.setEnabled(True)
        self.action_disconnect.setEnabled(False)
        self.connection_status_label.setText("Disconnected")
        self.device_info_label.setText("")
        
        # Disable tabs
        self.motor_control_tab.set_roboclaw(None)
        self.pid_tuning_tab.set_roboclaw(None)
        self.monitoring_tab.set_roboclaw(None)
        self.configuration_tab.set_roboclaw(None)
        
        self._stop_snapshot_thread()
        
        logger.info("Device disconnected")
    
    def _on_load_configuration(self):
        """Load device configuration from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON files (*.json);;All files (*.*)"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                
                # Load configuration into tabs
                self.configuration_tab.load_configuration(config)
                
                QMessageBox.information(self, "Success", "Configuration loaded successfully")
                logger.info(f"Configuration loaded from {filename}")
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load configuration: {e}")
                logger.error(f"Failed to load configuration: {e}")
    
    def _on_save_configuration(self):
        """Save device configuration to file"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "", "JSON files (*.json);;All files (*.*)"
        )
        
        if filename:
            try:
                config = self.configuration_tab.get_configuration()
                
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                
                QMessageBox.information(self, "Success", "Configuration saved successfully")
                logger.info(f"Configuration saved to {filename}")
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save configuration: {e}")
                logger.error(f"Failed to save configuration: {e}")
    
    def _on_autotune_pid(self):
        """Start PID auto-tuning"""
        if not self.device_connected:
            QMessageBox.warning(self, "Warning", "Please connect to a device first")
            return
        
        self.tab_widget.setCurrentWidget(self.pid_tuning_tab)
        self.pid_tuning_tab.start_autotuning()
    
    def _on_reset_encoders(self):
        """Reset encoder counts"""
        if not self.device_connected:
            QMessageBox.warning(self, "Warning", "Please connect to a device first")
            return
        
        reply = QMessageBox.question(
            self, "Reset Encoders", 
            "Are you sure you want to reset encoder counts to zero?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            if self.roboclaw.reset_encoders():
                QMessageBox.information(self, "Success", "Encoders reset successfully")
                logger.info("Encoders reset")
            else:
                QMessageBox.critical(self, "Error", "Failed to reset encoders")
                logger.error("Failed to reset encoders")
    
    def _on_emergency_stop(self):
        """Emergency stop all motors"""
        if self.device_connected and self.roboclaw:
            self.roboclaw.stop_motors()
            logger.warning("Emergency stop activated")
    
    def _on_toggle_debug(self):
        """Toggle debug mode"""
        debug_mode = self.debug_action.isChecked()
        self.settings.setValue("debug_mode", debug_mode)
        
        # Update logging level
        if debug_mode:
            logging.getLogger().setLevel(logging.DEBUG)
        else:
            logging.getLogger().setLevel(logging.INFO)
        
        logger.info(f"Debug mode {'enabled' if debug_mode else 'disabled'}")
    
    def _on_about(self):
        """Show about dialog"""
        QMessageBox.about(
            self, "About RoboClaw Motion Studio",
            """<h3>RoboClaw Motion Studio</h3>
            <p>Version 1.0.0</p>
            <p>A comprehensive GUI application for controlling and monitoring RoboClaw motor controllers.</p>
            <p>Features:</p>
            <ul>
            <li>Real-time motor monitoring</li>
            <li>PID tuning with auto-tuning</li>
            <li>Device configuration management</li>
            <li>Data visualization</li>
            </ul>
            <p>Built with PyQt6 for Linux systems.</p>"""
        )
    
    def _update_status(self):
        """Update status information"""
        if self.device_connected and self.roboclaw:
            # Update connection status with additional info
            try:
                temp = self.roboclaw.get_temperature()
                voltage = self.roboclaw.get_main_battery_voltage()
                
                status_parts = ["Connected"]
                if temp is not None:
                    status_parts.append(f"Temp: {temp:.1f}°C")
                if voltage is not None:
                    status_parts.append(f"Voltage: {voltage:.1f}V")
                
                self.connection_status_label.setText(" | ".join(status_parts))
            except Exception as e:
                logger.error(f"Status update error: {e}")
    
    def _update_monitoring(self):
        """Update monitoring data"""
        if self.device_connected and self.roboclaw:
            self.monitoring_tab.update_data()
    
    def show_progress(self, message: str, maximum: int = 0):
        """Show progress bar with message"""
        self.progress_bar.setMaximum(maximum)
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(True)
        self.status_bar.showMessage(message)
    
    def update_progress(self, value: int):
        """Update progress bar value"""
        self.progress_bar.setValue(value)
    
    def hide_progress(self):
        """Hide progress bar"""
        self.progress_bar.setVisible(False)
        self.status_bar.clearMessage()
    
    def _start_snapshot_thread(self):
        """Start the snapshot polling thread"""
        if self._snapshot_thread and self._snapshot_thread.is_alive():
            return
        self._snapshot_stop.clear()
        def run():
            while not self._snapshot_stop.is_set():
                if self.roboclaw:
                    snap = self.roboclaw.snapshot()
                    self._dispatch_snapshot(snap)
                time.sleep(self._snapshot_interval)
        self._snapshot_thread = threading.Thread(target=run, daemon=True)
        self._snapshot_thread.start()
    
    def _stop_snapshot_thread(self):
        """Stop the snapshot polling thread"""
        if self._snapshot_thread:
            self._snapshot_stop.set()
            self._snapshot_thread.join(timeout=1.0)
            self._snapshot_thread = None
    
    def _dispatch_snapshot(self, snap: dict):
        """Dispatch snapshot data to update methods"""
        # invoked from thread; use singleShot to update UI in main thread
        QTimer.singleShot(0, lambda s=snap: self._apply_snapshot(s))
    
    def _apply_snapshot(self, snap: dict):
        """Apply the snapshot data to update UI components"""
        try:
            self.monitoring_tab.update_from_snapshot(snap)
            self.motor_control_tab.update_from_snapshot(snap)
            # status bar quick info
            if 'mbatt' in snap and 'temp' in snap:
                self.connection_status_label.setText(f"V:{snap.get('mbatt',0):.1f} Temp:{snap.get('temp',0):.1f}C Errors:{','.join(snap.get('error_list', []))}")
        except Exception:
            pass
    
    def _set_monitoring(self, enable: bool):
        """Enable or disable external monitoring updates"""
        if enable:
            self.monitoring_tab.enable_external()
        else:
            self.monitoring_tab.disable_external()
