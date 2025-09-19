"""
Robust RoboClaw packet protocol implementation (manual compliant)
Derived from TeensyV2 RoboClaw C++ library patterns.
"""
from __future__ import annotations
import serial, time, struct, threading
from typing import Optional, Tuple, List, Dict
import logging

log = logging.getLogger(__name__)

# ACK byte per manual
_ACK = 0xFF
_MAX_RETRY = 2  # Matches MAXRETRY in C++ (2 plus initial try = 3 attempts)
_RETRY_DELAY = 0.010  # 10ms per manual between retries

# Command constants (subset + extendable)
class CMD:
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

class RoboClawError(Exception):
    pass

class AckError(RoboClawError):
    pass

class CrcError(RoboClawError):
    pass

class TimeoutError(RoboClawError):
    pass

class RoboClawProtocol:
    def __init__(self, port: str, baud: int = 115200, address: int = 0x80, timeout: float = 0.02):
        self.port_path = port
        self.baud = baud
        self.address = address
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._crc = 0
        self.ack_timeout = 0.2  # extend to 200ms
        self.stop_pacing = 0.005  # 5ms between stop packets

    # --- CRC ---
    def _crc_clear(self):
        self._crc = 0
    def _crc_update(self, b: int):
        self._crc ^= (b & 0xFF) << 8
        for _ in range(8):
            if self._crc & 0x8000:
                self._crc = ((self._crc << 1) ^ 0x1021) & 0xFFFF
            else:
                self._crc = (self._crc << 1) & 0xFFFF
    def _crc_value(self) -> int:
        return self._crc & 0xFFFF

    # --- Connection ---
    def connect(self):
        if self._ser and self._ser.is_open:
            return True
        with self._lock:
            if self._ser and self._ser.is_open:
                return True
            self._ser = serial.Serial(self.port_path, self.baud, timeout=self.timeout, bytesize=8, parity='N', stopbits=1)
            time.sleep(0.2)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        return True
    def close(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()

    # --- Low level write (write_n analogue) ---
    def _write_packet(self, *data_bytes: int):
        if not self._ser or not self._ser.is_open:
            raise RoboClawError("Serial not open")
        with self._lock:
            # Removed automatic input buffer reset to avoid discarding late ACK
            self._crc_clear()
            out = bytearray()
            for b in data_bytes:
                b &= 0xFF
                self._crc_update(b)
                out.append(b)
            crc = self._crc_value()
            out.append((crc >> 8) & 0xFF)
            out.append(crc & 0xFF)
            self._ser.write(out)
            self._ser.flush()
            log.debug("TX: %s", ' '.join(f"{x:02X}" for x in out))
            time.sleep(0.001)  # slight device processing gap
            deadline = time.monotonic() + self.ack_timeout
            stray_bytes = bytearray()
            cmd_hex = f"0x{data_bytes[1]:02X}" if len(data_bytes) > 1 else "?"
            while time.monotonic() < deadline:
                b = self._ser.read(1)
                if not b:
                    continue
                if b[0] == _ACK:
                    if stray_bytes:
                        log.debug("Stray before ACK for %s: %s", cmd_hex, ' '.join(f"{x:02X}" for x in stray_bytes))
                    log.debug("ACK OK for %s", cmd_hex)
                    return
                stray_bytes.extend(b)
            if stray_bytes:
                log.warning("No ACK for %s; stray bytes: %s", cmd_hex, ' '.join(f"{x:02X}" for x in stray_bytes))
            raise TimeoutError("ACK timeout")

    def _write_retry(self, *data_bytes: int):
        attempt = 0
        while True:
            try:
                self._write_packet(*data_bytes)
                return True
            except (AckError, TimeoutError) as e:
                if attempt >= _MAX_RETRY:
                    log.error("Write failed after retries: %s", e)
                    raise
                time.sleep(_RETRY_DELAY)
                attempt += 1

    # --- Low level read (ReadX analogue) ---
    def _read_command(self, cmd: int, expected_len: int) -> bytes:
        if not self._ser or not self._ser.is_open:
            raise RoboClawError("Serial not open")
        attempt = 0
        while True:
            try:
                self._ser.reset_input_buffer()
                self._crc_clear()
                # send header
                header = [self.address, cmd]
                for b in header:
                    self._crc_update(b)
                    self._ser.write(bytes([b]))
                # send CRC
                crc = self._crc_value()
                self._ser.write(bytes([(crc >> 8) & 0xFF, crc & 0xFF]))
                self._ser.flush()
                # read response + crc
                need = expected_len + 2
                buf = self._ser.read(need)
                if len(buf) != need:
                    raise TimeoutError("Response timeout")
                data = buf[:-2]
                rx_crc = (buf[-2] << 8) | buf[-1]
                self._crc_clear()
                for b in header:
                    self._crc_update(b)
                for b in data:
                    self._crc_update(b)
                if self._crc_value() != rx_crc:
                    raise CrcError(f"CRC mismatch calc=0x{self._crc_value():04X} rx=0x{rx_crc:04X}")
                log.debug("RX: %s", ' '.join(f"{x:02X}" for x in buf))
                return data
            except (TimeoutError, CrcError) as e:
                if attempt >= _MAX_RETRY:
                    log.error("Read failed after retries: %s", e)
                    raise
                time.sleep(_RETRY_DELAY)
                attempt += 1

    # --- Public motion & config API ---
    def forward_m1(self, speed: int):
        speed = max(0, min(127, speed))
        self._write_retry(self.address, CMD.M1FORWARD, speed)
    def backward_m1(self, speed: int):
        speed = max(0, min(127, speed))
        self._write_retry(self.address, CMD.M1BACKWARD, speed)
    def forward_m2(self, speed: int):
        speed = max(0, min(127, speed))
        self._write_retry(self.address, CMD.M2FORWARD, speed)
    def backward_m2(self, speed: int):
        speed = max(0, min(127, speed))
        self._write_retry(self.address, CMD.M2BACKWARD, speed)
    def duty_m1(self, duty_signed_percent: float):
        p = max(-100.0, min(100.0, duty_signed_percent))
        raw = int(p / 100.0 * 32767) & 0xFFFF
        self._write_retry(self.address, CMD.M1DUTY, (raw >> 8) & 0xFF, raw & 0xFF)
    def duty_m2(self, duty_signed_percent: float):
        p = max(-100.0, min(100.0, duty_signed_percent))
        raw = int(p / 100.0 * 32767) & 0xFFFF
        self._write_retry(self.address, CMD.M2DUTY, (raw >> 8) & 0xFF, raw & 0xFF)
    def set_velocity_pid_m1(self, kp: float, ki: float, kd: float, qpps: int):
        kd_raw = int(kd * 65536)
        kp_raw = int(kp * 65536)
        ki_raw = int(ki * 65536)
        pkt = struct.pack('>IIII', kd_raw, kp_raw, ki_raw, qpps)
        self._write_retry(self.address, CMD.SETM1PID, *pkt)
    def set_velocity_pid_m2(self, kp: float, ki: float, kd: float, qpps: int):
        kd_raw = int(kd * 65536)
        kp_raw = int(kp * 65536)
        ki_raw = int(ki * 65536)
        pkt = struct.pack('>IIII', kd_raw, kp_raw, ki_raw, qpps)
        self._write_retry(self.address, CMD.SETM2PID, *pkt)
    def read_velocity_pid_m1(self) -> Tuple[float,float,float,int]:
        data = self._read_command(CMD.READM1PID, 16)
        kd_raw, kp_raw, ki_raw, qpps = struct.unpack('>IIII', data)
        return kp_raw/65536.0, ki_raw/65536.0, kd_raw/65536.0, qpps
    def read_velocity_pid_m2(self) -> Tuple[float,float,float,int]:
        data = self._read_command(CMD.READM2PID, 16)
        kd_raw, kp_raw, ki_raw, qpps = struct.unpack('>IIII', data)
        return kp_raw/65536.0, ki_raw/65536.0, kd_raw/65536.0, qpps
    def read_version(self) -> str:
        """Read firmware version string.
        Strategy:
        1. Try legacy (no header CRC) pattern used by reference C++ lib.
        2. If that times out, try packet form with header CRC appended.
        In both cases CRC is validated over (address, cmd, bytes including null terminator).
        """
        if not self._ser or not self._ser.is_open:
            raise RoboClawError("Serial not open")
        def _attempt(send_crc: bool) -> str:
            self._crc_clear()
            # send header
            for b in (self.address, CMD.GETVERSION):
                self._crc_update(b)
                self._ser.write(bytes([b]))
            if send_crc:
                crc = self._crc_value()
                self._ser.write(bytes([(crc>>8)&0xFF, crc & 0xFF]))
            self._ser.flush()
            name_bytes = bytearray()
            # collect up to 48 bytes + null
            start = time.monotonic()
            while len(name_bytes) < 48:
                ch = self._ser.read(1)
                if ch:
                    val = ch[0]
                    self._crc_update(val)
                    if val == 0:  # terminator
                        # read CRC (always present after terminator)
                        crc_hi = self._ser.read(1)
                        crc_lo = self._ser.read(1)
                        if len(crc_hi)!=1 or len(crc_lo)!=1:
                            raise TimeoutError("Version CRC bytes missing")
                        rx_crc = (crc_hi[0]<<8)|crc_lo[0]
                        if self._crc_value()!=rx_crc:
                            raise CrcError("Version CRC mismatch")
                        return name_bytes.decode(errors='ignore')
                    else:
                        name_bytes.append(val)
                else:
                    if time.monotonic() - start > 0.5:  # 500ms overall budget per attempt
                        raise TimeoutError("Version timeout")
            raise TimeoutError("Version string too long / no terminator")
        attempt = 0
        while True:
            try:
                # try without CRC first
                try:
                    return _attempt(False)
                except (TimeoutError, CrcError):
                    # clear any pending
                    self._ser.reset_input_buffer()
                    return _attempt(True)
            except (TimeoutError, CrcError):
                if attempt >= _MAX_RETRY:
                    raise
                time.sleep(_RETRY_DELAY)
                attempt += 1
    def read_main_battery_voltage(self) -> float:
        data = self._read_command(CMD.GETMBATT, 2)
        val = struct.unpack('>H', data)[0]
        return val/10.0
    def read_logic_battery_voltage(self) -> float:
        data = self._read_command(CMD.GETLBATT, 2)
        return struct.unpack('>H', data)[0]/10.0
    def read_temperature(self) -> float:
        data = self._read_command(CMD.GETTEMP, 2)
        return struct.unpack('>H', data)[0]/10.0
    def read_currents(self) -> Tuple[float, float]:
        """Return (m1_current_amps, m2_current_amps). Each value in 10mA units per manual."""
        data = self._read_command(CMD.GETCURRENTS, 4)
        m1_raw, m2_raw = struct.unpack('>HH', data)
        return m1_raw / 100.0, m2_raw / 100.0
    def read_enc_m1(self) -> Tuple[int,int]:
        data = self._read_command(CMD.GETM1ENC, 5)
        enc = struct.unpack('>I', data[:4])[0]
        status = data[4]
        return enc, status
    def read_enc_m2(self) -> Tuple[int,int]:
        data = self._read_command(CMD.GETM2ENC, 5)
        enc = struct.unpack('>I', data[:4])[0]
        status = data[4]
        return enc, status
    def reset_encoders(self):
        self._write_retry(self.address, CMD.RESETENC)
    def read_error_status(self) -> int:
        data = self._read_command(CMD.GETERROR, 4)
        return struct.unpack('>I', data)[0] & 0x3FFFFFFF
    def stop_all(self):
        # Minimal, reliable stop: zero duty then zero speed (qpps) with pacing
        seq = [
            (self.duty_m1, 0),
            (self.duty_m2, 0),
            (self.speed_m1, 0),
            (self.speed_m2, 0)
        ]
        for fn, val in seq:
            try:
                fn(val)
            except Exception as e:
                log.warning("Stop step %s failed: %s", fn.__name__, e)
            time.sleep(self.stop_pacing)
    def speed_m1(self, qpps: int):
        # Signed 32-bit value
        val = int(qpps) & 0xFFFFFFFF
        pkt = [(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
        self._write_retry(self.address, CMD.M1SPEED, *pkt)
    def speed_m2(self, qpps: int):
        val = int(qpps) & 0xFFFFFFFF
        pkt = [(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
        self._write_retry(self.address, CMD.M2SPEED, *pkt)
    def read_speed_m1(self):
        data = self._read_command(CMD.GETM1SPEED, 5)
        return struct.unpack('>I', data[:4])[0], data[4]
    def read_speed_m2(self):
        data = self._read_command(CMD.GETM2SPEED, 5)
        return struct.unpack('>I', data[:4])[0], data[4]
    def read_pwms(self):
        data = self._read_command(CMD.GETPWMS, 4)
        return struct.unpack('>hh', data)

__all__ = [
    'RoboClawProtocol','RoboClawError','AckError','CrcError','TimeoutError','CMD'
]
