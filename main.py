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
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("RoboClaw Tools")
    
    # Set application icon if available
    icon_path = os.path.join(os.path.dirname(__file__), 'resources', 'icons', 'app_icon.png')
    if os.path.exists(icon_path):
        from PyQt6.QtGui import QIcon
        app.setWindowIcon(QIcon(icon_path))
    
    window = MainWindow()
    window.show()
    
    return app.exec()

if __name__ == "__main__":
    sys.exit(main())
