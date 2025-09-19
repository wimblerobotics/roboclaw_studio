#!/usr/bin/env python3
"""
RoboClaw Motion Studio Clone
A comprehensive GUI application for controlling and monitoring RoboClaw motor controllers.
"""

import sys
import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer
from roboclaw_motion_studio.main_window import MainWindow

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("RoboClaw Motion Studio")
    app.setApplicationVersion("1.1.0")
    app.setOrganizationName("RoboClaw Tools")
    
    # Set application icon if available
    icon_path = os.path.join(os.path.dirname(__file__), 'resources', 'icons', 'app_icon.png')
    if os.path.exists(icon_path):
        from PyQt6.QtGui import QIcon
        app.setWindowIcon(QIcon(icon_path))
    
    # Theme
    settings_path = os.path.join(os.path.expanduser('~'), '.config', 'RoboClawTools', 'MotionStudio.conf')
    # Simple dark theme option read via env or placeholder
    if os.environ.get('ROBOCLAW_THEME','').lower() == 'dark':
        app.setStyleSheet("""
            QWidget { background-color: #232629; color: #ddd; }
            QGroupBox { border: 1px solid #444; margin-top: 6px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding:0 4px; }
            QPushButton { background:#444; border:1px solid #555; padding:4px; }
            QPushButton:hover { background:#555; }
            QTabBar::tab:selected { background:#444; }
            QStatusBar { background:#1e1e1e; }
        """)
    
    window = MainWindow()
    window.show()
    
    return app.exec()

if __name__ == "__main__":
    sys.exit(main())
