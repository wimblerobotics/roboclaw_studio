"""
RoboClaw communication module adapted for Linux
Based on the RoboClaw library with Linux serial communication
"""

import serial
import time
import struct
from typing import Optional, Tuple, List
import logging

logger = logging.getLogger(__name__)

class RoboClawLinux:
    """Linux-compatible RoboClaw controller interface"""
    
    # Command constants
    M1FORWARD = 0
    M1BACKWARD = 1
    SETMINMB = 2
    SETMAXMB = 3
    M2FORWARD = 4
    M2BACKWARD = 5
    M17BIT = 6
    M27BIT = 7
    MIXEDFORWARD = 8
    MIXEDBACKWARD = 9
    MIXEDRIGHT = 10
    MIXEDLEFT = 11
    MIXEDFB = 12
    MIXEDLR = 13
    GETM1ENC = 16
    GETM2ENC = 17
    GETM1SPEED = 18
    GETM2SPEED = 19
    RESETENC = 20
    GETVERSION = 21
    SETM1ENCCOUNT = 22
    SETM2ENCCOUNT = 23
    GETMBATT = 24
    GETLBATT = 25
    SETMINLB = 26
    SETMAXLB = 27
    SETM1PID = 28
    SETM2PID = 29
    GETM1ISPEED = 30
    GETM2ISPEED = 31
    M1DUTY = 32
    M2DUTY = 33
    MIXEDDUTY = 34
    M1SPEED = 35
    M2SPEED = 36
    MIXEDSPEED = 37
    M1SPEEDACCEL = 38
    M2SPEEDACCEL = 39
    MIXEDSPEEDACCEL = 40
    M1SPEEDDIST = 41
    M2SPEEDDIST = 42
    MIXEDSPEEDDIST = 43
    M1SPEEDACCELDIST = 44
    M2SPEEDACCELDIST = 45
    MIXEDSPEEDACCELDIST = 46
    GETBUFFERS = 47
    GETPWMS = 48
    GETCURRENTS = 49
    MIXEDSPEED2ACCEL = 50
    MIXEDSPEED2ACCELDIST = 51
    M1DUTYACCEL = 52
    M2DUTYACCEL = 53
    MIXEDDUTYACCEL = 54
    READM1PID = 55
    READM2PID = 56
    SETMAINVOLTAGES = 57
    SETLOGICVOLTAGES = 58
    GETMINMAXMAINVOLTAGES = 59
    GETMINMAXLOGICVOLTAGES = 60
    SETM1POSPID = 61
    SETM2POSPID = 62
    READM1POSPID = 63
    READM2POSPID = 64
    GETTEMP = 82
    GETERROR = 90
    
    def __init__(self, port: str, timeout: float = 1.0):
        """Initialize RoboClaw communication
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            timeout: Communication timeout in seconds
        """
        self.port = port
        self.timeout = timeout
        self.serial = None
        self.address = 0x80  # Default address
        
    def connect(self, baudrate: int = 38400) -> bool:
        """Connect to RoboClaw device
        
        Args:
            baudrate: Communication baud rate
            
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            logger.info(f"Connected to RoboClaw on {self.port} at {baudrate} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to RoboClaw: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from RoboClaw device"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("Disconnected from RoboClaw")
    
    def _calculate_crc(self, data: bytes) -> int:
        """Calculate CRC16 checksum for data packet"""
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
    
    def _write_command(self, address: int, command: int, *args) -> bool:
        """Write command to RoboClaw with CRC"""
        if not self.serial or not self.serial.is_open:
            return False
        
        packet = bytearray([address, command])
        packet.extend(args)
        
        crc = self._calculate_crc(packet)
        packet.extend([(crc >> 8) & 0xFF, crc & 0xFF])
        
        try:
            self.serial.write(packet)
            response = self.serial.read(1)
            return len(response) == 1 and response[0] == 0xFF
        except Exception as e:
            logger.error(f"Command write failed: {e}")
            return False
    
    def _read_response(self, address: int, command: int, response_length: int) -> Optional[bytes]:
        """Read response from RoboClaw with CRC validation"""
        if not self.serial or not self.serial.is_open:
            return None
        
        try:
            self.serial.write(bytes([address, command]))
            response = self.serial.read(response_length + 2)  # +2 for CRC
            
            if len(response) != response_length + 2:
                return None
            
            data = response[:-2]
            received_crc = (response[-2] << 8) | response[-1]
            
            packet = bytes([address, command]) + data
            calculated_crc = self._calculate_crc(packet)
            
            if received_crc == calculated_crc:
                return data
            return None
        except Exception as e:
            logger.error(f"Response read failed: {e}")
            return None
    
    def get_version(self, address: Optional[int] = None) -> Optional[str]:
        """Get firmware version string"""
        addr = address or self.address
        data = self._read_response(addr, self.GETVERSION, 32)
        if data:
            try:
                return data.decode('ascii').rstrip('\x00')
            except UnicodeDecodeError:
                return None
        return None
    
    def get_main_battery_voltage(self, address: Optional[int] = None) -> Optional[float]:
        """Get main battery voltage in volts"""
        addr = address or self.address
        data = self._read_response(addr, self.GETMBATT, 2)
        if data:
            voltage_raw = struct.unpack('>H', data)[0]
            return voltage_raw / 10.0  # Convert to volts
        return None
    
    def get_logic_battery_voltage(self, address: Optional[int] = None) -> Optional[float]:
        """Get logic battery voltage in volts"""
        addr = address or self.address
        data = self._read_response(addr, self.GETLBATT, 2)
        if data:
            voltage_raw = struct.unpack('>H', data)[0]
            return voltage_raw / 10.0  # Convert to volts
        return None
    
    def get_temperature(self, address: Optional[int] = None) -> Optional[float]:
        """Get temperature in Celsius"""
        addr = address or self.address
        data = self._read_response(addr, self.GETTEMP, 2)
        if data:
            temp_raw = struct.unpack('>H', data)[0]
            return temp_raw / 10.0  # Convert to Celsius
        return None
    
    def get_encoder_m1(self, address: Optional[int] = None) -> Optional[Tuple[int, int]]:
        """Get M1 encoder count and status"""
        addr = address or self.address
        data = self._read_response(addr, self.GETM1ENC, 5)
        if data:
            encoder_count = struct.unpack('>I', data[:4])[0]
            status = data[4]
            return encoder_count, status
        return None
    
    def get_encoder_m2(self, address: Optional[int] = None) -> Optional[Tuple[int, int]]:
        """Get M2 encoder count and status"""
        addr = address or self.address
        data = self._read_response(addr, self.GETM2ENC, 5)
        if data:
            encoder_count = struct.unpack('>I', data[:4])[0]
            status = data[4]
            return encoder_count, status
        return None
    
    def get_speed_m1(self, address: Optional[int] = None) -> Optional[Tuple[int, int]]:
        """Get M1 speed and status"""
        addr = address or self.address
        data = self._read_response(addr, self.GETM1SPEED, 5)
        if data:
            speed = struct.unpack('>I', data[:4])[0]
            status = data[4]
            return speed, status
        return None
    
    def get_speed_m2(self, address: Optional[int] = None) -> Optional[Tuple[int, int]]:
        """Get M2 speed and status"""
        addr = address or self.address
        data = self._read_response(addr, self.GETM2SPEED, 5)
        if data:
            speed = struct.unpack('>I', data[:4])[0]
            status = data[4]
            return speed, status
        return None
    
    def get_currents(self, address: Optional[int] = None) -> Optional[Tuple[float, float]]:
        """Get motor currents in Amps"""
        addr = address or self.address
        data = self._read_response(addr, self.GETCURRENTS, 4)
        if data:
            current1_raw, current2_raw = struct.unpack('>HH', data)
            # Convert to Amps (assuming 10mA per unit)
            current1 = current1_raw / 100.0
            current2 = current2_raw / 100.0
            return current1, current2
        return None
    
    def get_pwm_values(self, address: Optional[int] = None) -> Optional[Tuple[int, int]]:
        """Get PWM values for both motors"""
        addr = address or self.address
        data = self._read_response(addr, self.GETPWMS, 4)
        if data:
            pwm1, pwm2 = struct.unpack('>hh', data)  # Signed values
            return pwm1, pwm2
        return None
    
    def read_velocity_pid_m1(self, address: Optional[int] = None) -> Optional[Tuple[float, float, float, int]]:
        """Read velocity PID parameters for M1"""
        addr = address or self.address
        data = self._read_response(addr, self.READM1PID, 16)
        if data:
            kp_raw, ki_raw, kd_raw, qpps = struct.unpack('>IIII', data)
            kp = kp_raw / 65536.0
            ki = ki_raw / 65536.0
            kd = kd_raw / 65536.0
            return kp, ki, kd, qpps
        return None
    
    def read_velocity_pid_m2(self, address: Optional[int] = None) -> Optional[Tuple[float, float, float, int]]:
        """Read velocity PID parameters for M2"""
        addr = address or self.address
        data = self._read_response(addr, self.READM2PID, 16)
        if data:
            kp_raw, ki_raw, kd_raw, qpps = struct.unpack('>IIII', data)
            kp = kp_raw / 65536.0
            ki = ki_raw / 65536.0
            kd = kd_raw / 65536.0
            return kp, ki, kd, qpps
        return None
    
    def set_velocity_pid_m1(self, kp: float, ki: float, kd: float, qpps: int, address: Optional[int] = None) -> bool:
        """Set velocity PID parameters for M1"""
        addr = address or self.address
        kp_raw = int(kp * 65536)
        ki_raw = int(ki * 65536)
        kd_raw = int(kd * 65536)
        
        data = struct.pack('>IIII', kp_raw, ki_raw, kd_raw, qpps)
        return self._write_command(addr, self.SETM1PID, *data)
    
    def set_velocity_pid_m2(self, kp: float, ki: float, kd: float, qpps: int, address: Optional[int] = None) -> bool:
        """Set velocity PID parameters for M2"""
        addr = address or self.address
        kp_raw = int(kp * 65536)
        ki_raw = int(ki * 65536)
        kd_raw = int(kd * 65536)
        
        data = struct.pack('>IIII', kp_raw, ki_raw, kd_raw, qpps)
        return self._write_command(addr, self.SETM2PID, *data)
    
    def forward_m1(self, speed: int, address: Optional[int] = None) -> bool:
        """Drive M1 forward at specified speed (0-127)"""
        addr = address or self.address
        return self._write_command(addr, self.M1FORWARD, speed)
    
    def backward_m1(self, speed: int, address: Optional[int] = None) -> bool:
        """Drive M1 backward at specified speed (0-127)"""
        addr = address or self.address
        return self._write_command(addr, self.M1BACKWARD, speed)
    
    def forward_m2(self, speed: int, address: Optional[int] = None) -> bool:
        """Drive M2 forward at specified speed (0-127)"""
        addr = address or self.address
        return self._write_command(addr, self.M2FORWARD, speed)
    
    def backward_m2(self, speed: int, address: Optional[int] = None) -> bool:
        """Drive M2 backward at specified speed (0-127)"""
        addr = address or self.address
        return self._write_command(addr, self.M2BACKWARD, speed)
    
    def drive_m1_m2(self, speed1: int, speed2: int, address: Optional[int] = None) -> bool:
        """Drive both motors with signed speed values (-127 to 127)"""
        addr = address or self.address
        data = struct.pack('>bb', speed1, speed2)
        return self._write_command(addr, self.MIXEDFB, *data)
    
    def stop_motors(self, address: Optional[int] = None) -> bool:
        """Stop both motors"""
        return self.drive_m1_m2(0, 0, address)
    
    def reset_encoders(self, address: Optional[int] = None) -> bool:
        """Reset encoder counts to zero"""
        addr = address or self.address
        return self._write_command(addr, self.RESETENC)
    
    def get_error_status(self, address: Optional[int] = None) -> Optional[int]:
        """Get error status flags"""
        addr = address or self.address
        data = self._read_response(addr, self.GETERROR, 4)
        if data:
            return struct.unpack('>I', data)[0]
        return None
