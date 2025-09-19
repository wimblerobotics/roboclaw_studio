"""
RoboClaw Motion Studio Clone Package
"""

__version__ = "1.1.0"
__author__ = "RoboClaw Tools"
__description__ = "A comprehensive GUI application for controlling and monitoring RoboClaw motor controllers"

from .roboclaw_protocol import RoboClawProtocol
__all__ = ['RoboClawProtocol']
