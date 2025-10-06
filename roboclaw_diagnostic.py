#!/usr/bin/env python3
"""
RoboClaw Connection Diagnostic Tool
Tests serial communication with RoboClaw devices
"""

import serial
import time
import struct
import sys
import glob

def find_serial_ports():
    """Find available serial ports"""
    ports = []
    patterns = ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyS*']
    
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    
    return sorted(ports)

def calculate_crc(data):
    """Calculate CRC16 for RoboClaw command"""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def test_roboclaw_communication(port, baudrate=38400, address=0x80):
    """Test basic communication with RoboClaw"""
    print(f"\n=== Testing {port} at {baudrate} baud ===")
    
    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print(f"✓ Serial port opened successfully")
        
        # Test 1: Get firmware version (command 21)
        print(f"  Testing firmware version command...")
        
        # Send get version command
        command = [address, 21]  # GET_VERSION command
        crc = calculate_crc(command)
        packet = command + [(crc >> 8) & 0xFF, crc & 0xFF]
        
        ser.write(bytes(packet))
        time.sleep(0.1)
        
        # Read response
        response = ser.read(50)  # Version string can be up to 48 chars + CRC
        
        if len(response) > 2:
            # Try to decode version string
            version_data = response[:-2]  # Remove CRC
            try:
                version = version_data.decode('ascii').rstrip('\x00')
                if version:
                    print(f"✓ Firmware version: {version}")
                    return True
                else:
                    print(f"✗ Empty version response")
            except UnicodeDecodeError:
                print(f"✗ Invalid version response: {response.hex()}")
        else:
            print(f"✗ No response or invalid response length: {len(response)} bytes")
            if len(response) > 0:
                print(f"   Response: {response.hex()}")
        
        # Test 2: Try different addresses
        if address == 0x80:
            print(f"  Trying other common addresses...")
            for test_addr in [0x81, 0x82, 0x83, 0x84]:
                command = [test_addr, 21]
                crc = calculate_crc(command)
                packet = command + [(crc >> 8) & 0xFF, crc & 0xFF]
                
                ser.write(bytes(packet))
                time.sleep(0.1)
                response = ser.read(50)
                
                if len(response) > 2:
                    version_data = response[:-2]
                    try:
                        version = version_data.decode('ascii').rstrip('\x00')
                        if version:
                            print(f"✓ Found device at address 0x{test_addr:02X}: {version}")
                            ser.close()
                            return True
                    except UnicodeDecodeError:
                        pass
        
        ser.close()
        return False
        
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except PermissionError as e:
        print(f"✗ Permission error: {e}")
        print(f"   Try: sudo chmod 666 {port}")
        print(f"   Or add user to dialout group and logout/login")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def test_baud_rates(port, address=0x80):
    """Test multiple baud rates"""
    baud_rates = [38400, 115200, 19200, 9600, 57600, 230400]
    
    print(f"\n=== Testing multiple baud rates on {port} ===")
    
    for baud in baud_rates:
        if test_roboclaw_communication(port, baud, address):
            print(f"\n🎉 SUCCESS: RoboClaw found on {port} at {baud} baud, address 0x{address:02X}")
            return port, baud, address
    
    return None, None, None

def main():
    print("RoboClaw Connection Diagnostic Tool")
    print("=" * 50)
    
    # Find serial ports
    ports = find_serial_ports()
    
    if not ports:
        print("❌ No serial ports found!")
        print("\nTroubleshooting:")
        print("1. Make sure RoboClaw is connected via USB")
        print("2. Check USB cable")
        print("3. Try different USB port")
        return 1
    
    print(f"Found {len(ports)} serial ports:")
    for port in ports:
        print(f"  {port}")
    
    # Test each port
    for port in ports:
        if '/dev/ttyS' in port and int(port[-1]) > 3:
            continue  # Skip high-numbered ttyS ports (usually not real)
        
        port_result, baud_result, addr_result = test_baud_rates(port)
        if port_result:
            print(f"\n✅ SOLUTION FOUND!")
            print(f"   Port: {port_result}")
            print(f"   Baud Rate: {baud_result}")
            print(f"   Address: 0x{addr_result:02X}")
            print(f"\nUse these settings in the RoboClaw Motion Studio application.")
            return 0
    
    print(f"\n❌ No RoboClaw devices found on any port!")
    print(f"\nTroubleshooting checklist:")
    print(f"1. Verify RoboClaw is powered on")
    print(f"2. Check USB cable connection")
    print(f"3. Try different USB cable")
    print(f"4. Check RoboClaw configuration (packet serial mode)")
    print(f"5. Verify device address (default is usually 0x80)")
    print(f"6. Try manual configuration with Motion Studio software first")
    print(f"7. Check for driver issues: dmesg | tail")
    
    return 1

if __name__ == "__main__":
    sys.exit(main())
