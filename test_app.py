#!/usr/bin/env python3
"""
Test script to verify RoboClaw Motion Studio can start without a device connected
"""

import sys
import os

# Add the project directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_imports():
    """Test that all modules can be imported"""
    print("Testing imports...")
    
    try:
        import PyQt6
        print("✓ PyQt6 imported successfully")
    except ImportError as e:
        print(f"✗ PyQt6 import failed: {e}")
        return False
    
    try:
        import roboclaw_motion_studio
        print("✓ roboclaw_motion_studio package imported successfully")
    except ImportError as e:
        print(f"✗ roboclaw_motion_studio import failed: {e}")
        return False
    
    try:
        from roboclaw_motion_studio.main_window import MainWindow
        print("✓ MainWindow imported successfully")
    except ImportError as e:
        print(f"✗ MainWindow import failed: {e}")
        return False
    
    try:
        from roboclaw_motion_studio.roboclaw_linux import RoboClawLinux
        print("✓ RoboClawLinux imported successfully")
    except ImportError as e:
        print(f"✗ RoboClawLinux import failed: {e}")
        return False
    
    return True

def test_gui_creation():
    """Test that the GUI can be created (without showing it)"""
    print("\nTesting GUI creation...")
    
    try:
        from PyQt6.QtWidgets import QApplication
        from roboclaw_motion_studio.main_window import MainWindow
        
        # Create QApplication instance
        app = QApplication([])
        
        # Create main window
        window = MainWindow()
        print("✓ MainWindow created successfully")
        
        # Test that tabs are created
        tab_count = window.tab_widget.count()
        print(f"✓ Created {tab_count} tabs")
        
        if tab_count == 5:  # Should have 5 tabs
            print("✓ All expected tabs created")
        else:
            print(f"⚠ Expected 5 tabs, got {tab_count}")
        
        # Clean up
        window.close()
        app.quit()
        
        return True
        
    except Exception as e:
        print(f"✗ GUI creation failed: {e}")
        return False

def main():
    """Run all tests"""
    print("RoboClaw Motion Studio - System Test")
    print("=" * 40)
    
    # Test imports
    if not test_imports():
        print("\n❌ Import tests failed!")
        return 1
    
    # Test GUI creation
    if not test_gui_creation():
        print("\n❌ GUI creation tests failed!")
        return 1
    
    print("\n✅ All tests passed!")
    print("\nYou can now:")
    print("1. Run 'python main.py' to start the application")
    print("2. Use 'roboclaw-studio' command from terminal (after restart)")
    print("3. Find 'RoboClaw Motion Studio' in your application menu")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
