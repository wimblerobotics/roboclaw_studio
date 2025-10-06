"""
Robust RoboClaw packet protocol implementation (manual compliant)
Derived from TeensyV2 RoboClaw C++ library patterns.
"""
from __future__ import annotations
import serial, time, struct, threading
from typing import Optional, Tuple, List, Dict
import logging

log = logging.getLogger(__name__)

# Motion focused debug flags
MOTION_FOCUS = True  # Set True to suppress routine telemetry noise

# Suppress verbose RX logging for routine high-frequency telemetry reads
_QUIET_RX_CMDS = {
    24,  # GETMBATT
    25,  # GETLBATT
    49,  # GETCURRENTS
    82,  # GETTEMP
}

# ACK byte per manual
_ACK = 0xFF
_MAX_RETRY = 2  # Matches MAXRETRY in C++ (2 plus initial try = 3 attempts)
_RETRY_DELAY = 0.010  # 10ms per manual between retries
_RESYNC_DELAY = 0.011  # Minimum 11ms line idle after error per manual to resync

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
        self._last_activity = time.monotonic()
        self.watchdog_enabled = True
        # Diagnostics stats
        self._stats = {"writes":0, "write_errors":0, "reads":0, "read_errors":0}
        self.verbose_rx = False  # can be toggled at runtime
        self.motion_debug = True

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
            start = time.monotonic()
            self._crc_clear()
            out = bytearray()
            for b in data_bytes:
                b &= 0xFF
                self._crc_update(b)
                out.append(b)
            crc = self._crc_value()
            out.append((crc >> 8) & 0xFF)
            out.append(crc & 0xFF)
            cmd_id = data_bytes[1] if len(data_bytes) > 1 else None
            if self.motion_debug:
                log.info(f"WRITE start cmd=0x{cmd_id:02X} bytes={' '.join(f'{x:02X}' for x in out)}")
            self._ser.write(out)
            self._ser.flush()
            deadline = time.monotonic() + self.ack_timeout
            stray_bytes = bytearray()
            while time.monotonic() < deadline:
                b = self._ser.read(1)
                if not b:
                    continue
                if b[0] == _ACK:
                    dur = (time.monotonic()-start)*1000.0
                    self._stats["writes"] += 1
                    if self.motion_debug:
                        log.info(f"WRITE ack cmd=0x{cmd_id:02X} dur_ms={dur:.1f}")
                    return
                stray_bytes.extend(b)
            self._stats["write_errors"] += 1
            if self.motion_debug:
                log.warning(f"WRITE timeout cmd=0x{cmd_id:02X} stray={' '.join(f'{x:02X}' for x in stray_bytes) if stray_bytes else 'NONE'}")
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
                    # Final resync pause before propagating
                    time.sleep(_RESYNC_DELAY)
                    raise
                log.debug("Write error (%s) attempt %d -> resync delay %.3f s", e, attempt+1, _RESYNC_DELAY)
                time.sleep(_RESYNC_DELAY)
                attempt += 1

    # --- Low level read (ReadX analogue) ---
    def _read_command(self, cmd: int, expected_len: int) -> bytes:
        if not self._ser or not self._ser.is_open:
            raise RoboClawError("Serial not open")
        attempt = 0
        while True:
            try:
                with self._lock:
                    self._crc_clear()
                    header = [self.address, cmd]
                    for b in header:
                        self._crc_update(b)
                        self._ser.write(bytes([b]))
                    crc = self._crc_value()
                    self._ser.write(bytes([(crc >> 8) & 0xFF, crc & 0xFF]))
                    self._ser.flush()
                    need = expected_len + 2  # payload + crc
                    buf = bytearray()
                    start = time.monotonic()
                    while len(buf) < need:
                        chunk = self._ser.read(need - len(buf))
                        if chunk:
                            buf.extend(chunk)
                        else:
                            if time.monotonic() - start > self.timeout:
                                raise TimeoutError("Response timeout")
                    data = bytes(buf[:-2])
                    rx_crc = (buf[-2] << 8) | buf[-1]
                    self._crc_clear()
                    for b in header:
                        self._crc_update(b)
                    for b in data:
                        self._crc_update(b)
                    if self._crc_value() != rx_crc:
                        raise CrcError(f"CRC mismatch calc=0x{self._crc_value():04X} rx=0x{rx_crc:04X}")
                    if not MOTION_FOCUS or self.verbose_rx:
                        log.debug("RX: %s (cmd 0x%02X %.2f ms)", ' '.join(f"{x:02X}" for x in buf), cmd, (time.monotonic()-start)*1000.0)
                    self._stats["reads"] += 1
                    return data
            except (TimeoutError, CrcError) as e:
                self._stats["read_errors"] += 1
                if attempt >= _MAX_RETRY:
                    log.error("Read failed after retries: %s", e)
                    time.sleep(_RESYNC_DELAY)
                    raise
                log.debug("Read error (%s) attempt %d -> resync delay %.3f s", e, attempt+1, _RESYNC_DELAY)
                # Mandatory idle before retry to allow hardware to finish packet & realign
                time.sleep(_RESYNC_DELAY)
                # Drain any straggler bytes to realign framing
                try:
                    with self._lock:
                        waiting = getattr(self._ser, 'in_waiting', 0)
                        if waiting:
                            junk = self._ser.read(waiting)
                            if junk:
                                log.debug("Drained %d stray bytes: %s", len(junk), ' '.join(f"{x:02X}" for x in junk))
                except Exception:
                    pass
                attempt += 1

    # --- Public motion & config API ---
    def forward_m1(self, speed: int):
        """Drive M1 forward using duty cycle (more reliable than legacy forward command)"""
        if self.motion_debug:
            log.info(f"CMD forward_m1 requested speed={speed}")
        # Convert 0-127 speed to percentage and use duty command
        duty_percent = min(100.0, (speed / 127.0) * 100.0)
        self.duty_m1(duty_percent)
    
    def backward_m1(self, speed: int):
        """Drive M1 backward using duty cycle (more reliable than legacy backward command)"""
        if self.motion_debug:
            log.info(f"CMD backward_m1 requested speed={speed}")
        # Convert 0-127 speed to negative percentage and use duty command
        duty_percent = -min(100.0, (speed / 127.0) * 100.0)
        self.duty_m1(duty_percent)
    
    def forward_m2(self, speed: int):
        """Drive M2 forward using duty cycle (more reliable than legacy forward command)"""
        if self.motion_debug:
            log.info(f"CMD forward_m2 requested speed={speed}")
        duty_percent = min(100.0, (speed / 127.0) * 100.0)
        self.duty_m2(duty_percent)
    
    def backward_m2(self, speed: int):
        """Drive M2 backward using duty cycle (more reliable than legacy backward command)"""
        if self.motion_debug:
            log.info(f"CMD backward_m2 requested speed={speed}")
        duty_percent = -min(100.0, (speed / 127.0) * 100.0)
        self.duty_m2(duty_percent)
    def duty_m1(self, duty_signed_percent: float):
        if self.motion_debug:
            log.info(f"CMD duty_m1 duty={duty_signed_percent:.2f}%")
        p = max(-100.0, min(100.0, duty_signed_percent))
        raw = int(p / 100.0 * 32767) & 0xFFFF
        self._write_retry(self.address, CMD.M1DUTY, (raw >> 8) & 0xFF, raw & 0xFF)
        self._touch()
    
    def duty_m2(self, duty_signed_percent: float):
        if self.motion_debug:
            log.info(f"CMD duty_m2 duty={duty_signed_percent:.2f}%")
        p = max(-100.0, min(100.0, duty_signed_percent))
        raw = int(p / 100.0 * 32767) & 0xFFFF
        self._write_retry(self.address, CMD.M2DUTY, (raw >> 8) & 0xFF, raw & 0xFF)
        self._touch()
    def set_velocity_pid_m1(self, kp: float, ki: float, kd: float, qpps: int):
        kd_raw = int(kd * 65536)
        kp_raw = int(kp * 65536)
        ki_raw = int(ki * 65536)
        pkt = struct.pack('>IIII', kd_raw, kp_raw, ki_raw, qpps)
        self._write_retry(self.address, CMD.SETM1PID, *pkt)
        self._touch()
    def set_velocity_pid_m2(self, kp: float, ki: float, kd: float, qpps: int):
        kd_raw = int(kd * 65536)
        kp_raw = int(kp * 65536)
        ki_raw = int(ki * 65536)
        pkt = struct.pack('>IIII', kd_raw, kp_raw, ki_raw, qpps)
        self._write_retry(self.address, CMD.SETM2PID, *pkt)
        self._touch()
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
        In both cases CRC is validated over (address, command, bytes including null terminator).
        """
        if not self._ser or not self._ser.is_open:
            raise RoboClawError("Serial not open")
        def _attempt(send_crc: bool) -> str:
            with self._lock:
                self._crc_clear()
                for b in (self.address, CMD.GETVERSION):
                    self._crc_update(b)
                    self._ser.write(bytes([b]))
                if send_crc:
                    crc = self._crc_value()
                    self._ser.write(bytes([(crc>>8)&0xFF, crc & 0xFF]))
                self._ser.flush()
            name_bytes = bytearray()
            start = time.monotonic()
            while len(name_bytes) < 48:
                ch = self._ser.read(1)
                if ch:
                    val = ch[0]
                    self._crc_update(val)
                    if val == 0:
                        crc_hi = self._ser.read(1); crc_lo = self._ser.read(1)
                        if len(crc_hi)!=1 or len(crc_lo)!=1:
                            raise TimeoutError("Version CRC bytes missing")
                        rx_crc = (crc_hi[0]<<8)|crc_lo[0]
                        if self._crc_value()!=rx_crc:
                            raise CrcError("Version CRC mismatch")
                        return name_bytes.decode(errors='ignore')
                    else:
                        name_bytes.append(val)
                else:
                    if time.monotonic() - start > 0.5:
                        raise TimeoutError("Version timeout")
            raise TimeoutError("Version string too long / no terminator")
        attempt = 0
        while True:
            try:
                try:
                    return _attempt(False)
                except (TimeoutError, CrcError):
                    with self._lock:
                        waiting = getattr(self._ser, 'in_waiting', 0)
                        if waiting:
                            self._ser.read(waiting)
                    time.sleep(_RESYNC_DELAY)
                    return _attempt(True)
            except (TimeoutError, CrcError) as e:
                if attempt >= _MAX_RETRY:
                    raise
                log.debug("Version read error (%s) attempt %d -> resync delay %.3f s", e, attempt+1, _RESYNC_DELAY)
                time.sleep(_RESYNC_DELAY)
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
        self._touch()
    def read_error_status(self) -> int:
        data = self._read_command(CMD.GETERROR, 4)
        return struct.unpack('>I', data)[0] & 0x3FFFFFFF

    def read_speed_m1(self) -> Tuple[int, int]:
        """Read motor 1 speed in QPPS (pulses per second)."""
        data = self._read_command(CMD.GETM1ISPEED, 5)
        speed = struct.unpack('>i', data[:4])[0]
        status = data[4]
        return speed, status

    def read_speed_m2(self) -> Tuple[int, int]:
        """Read motor 2 speed in QPPS (pulses per second)."""
        data = self._read_command(CMD.GETM2ISPEED, 5)
        speed = struct.unpack('>i', data[:4])[0]
        status = data[4]
        return speed, status

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
        self._touch()
    def speed_m1(self, qpps: int):
        log.debug("CMD speed_m1 qpps=%d", qpps)
        # Signed 32-bit value
        val = int(qpps) & 0xFFFFFFFF
        pkt = [(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
        self._write_retry(self.address, CMD.M1SPEED, *pkt)
        self._touch()
    def speed_m2(self, qpps: int):
        log.debug("CMD speed_m2 qpps=%d", qpps)
        val = int(qpps) & 0xFFFFFFFF
        pkt = [(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
        self._write_retry(self.address, CMD.M2SPEED, *pkt)
        self._touch()
    def speed_accel_m1(self, accel: int, speed: int):
        log.debug("CMD speed_accel_m1 accel=%d speed=%d", accel, speed)
        accel &= 0xFFFFFFFF; speed &= 0xFFFFFFFF
        pkt = [(accel>>24)&0xFF,(accel>>16)&0xFF,(accel>>8)&0xFF,accel&0xFF,(speed>>24)&0xFF,(speed>>16)&0xFF,(speed>>8)&0xFF,speed&0xFF]
        self._write_retry(self.address, CMD.M1SPEEDACCEL, *pkt)
        self._touch()
    def speed_accel_m2(self, accel: int, speed: int):
        log.debug("CMD speed_accel_m2 accel=%d speed=%d", accel, speed)
        accel &= 0xFFFFFFFF; speed &= 0xFFFFFFFF
        pkt = [(accel>>24)&0xFF,(accel>>16)&0xFF,(accel>>8)&0xFF,accel&0xFF,(speed>>24)&0xFF,(speed>>16)&0xFF,(speed>>8)&0xFF,speed&0xFF]
        self._write_retry(self.address, CMD.M2SPEEDACCEL, *pkt)
        self._touch()
    def speed_accel_distance_m1(self, accel: int, speed: int, distance: int, buffer: int = 1):
        log.debug("CMD speed_accel_distance_m1 accel=%d speed=%d dist=%d buf=%d", accel, speed, distance, buffer)
        accel &= 0xFFFFFFFF; speed &= 0xFFFFFFFF; distance &= 0xFFFFFFFF; buffer &= 0x01
        pkt = [
            (accel>>24)&0xFF,(accel>>16)&0xFF,(accel>>8)&0xFF,accel&0xFF,
            (speed>>24)&0xFF,(speed>>16)&0xFF,(speed>>8)&0xFF,speed&0xFF,
            (distance>>24)&0xFF,(distance>>16)&0xFF,(distance>>8)&0xFF,distance&0xFF, buffer & 0x01]
        self._write_retry(self.address, CMD.M1SPEEDACCELDIST, *pkt)
        self._touch()
    def speed_accel_distance_m2(self, accel: int, speed: int, distance: int, buffer: int = 1):
        log.debug("CMD speed_accel_distance_m2 accel=%d speed=%d dist=%d buf=%d", accel, speed, distance, buffer)
        accel &= 0xFFFFFFFF; speed &= 0xFFFFFFFF; distance &= 0xFFFFFFFF; buffer &= 0x01
        pkt = [
            (accel>>24)&0xFF,(accel>>16)&0xFF,(accel>>8)&0xFF,accel&0xFF,
            (speed>>24)&0xFF,(speed>>16)&0xFF,(speed>>8)&0xFF,speed&0xFF,
            (distance>>24)&0xFF,(distance>>16)&0xFF,(distance>>8)&0xFF,distance&0xFF, buffer & 0x01]
        self._write_retry(self.address, CMD.M2SPEEDACCELDIST, *pkt)
        self._touch()

    def _touch(self):
        """Update the last activity timestamp for watchdog purposes."""
        self._last_activity = time.monotonic()

    def set_position_pid_m1(self, kp: float, ki: float, kd: float, ilimit: int, deadzone: int, min_pos: int, max_pos: int):
        # position PID scaling per manual: kp, ki, kd -> value*1024
        vals = [int(kd*1024), int(kp*1024), int(ki*1024), ilimit, deadzone, min_pos, max_pos]
        pkt = struct.pack('>IIIIIII', *[v & 0xFFFFFFFF for v in vals])
        self._write_retry(self.address, CMD.SETM1POSPID, *pkt)
        self._touch()
    def set_position_pid_m2(self, kp: float, ki: float, kd: float, ilimit: int, deadzone: int, min_pos: int, max_pos: int):
        vals = [int(kd*1024), int(kp*1024), int(ki*1024), ilimit, deadzone, min_pos, max_pos]
        pkt = struct.pack('>IIIIIII', *[v & 0xFFFFFFFF for v in vals])
        self._write_retry(self.address, CMD.SETM2POSPID, *pkt)
        self._touch()
    def read_position_pid_m1(self):
        data = self._read_command(CMD.READM1POSPID, 28)
        kd_raw,kp_raw,ki_raw,ilim,dead,imin,imax = struct.unpack('>IIIIIII', data)
        return kp_raw/1024.0, ki_raw/1024.0, kd_raw/1024.0, ilim, dead, imin, imax
    def read_position_pid_m2(self):
        data = self._read_command(CMD.READM2POSPID, 28)
        kd_raw,kp_raw,ki_raw,ilim,dead,imin,imax = struct.unpack('>IIIIIII', data)
        return kp_raw/1024.0, ki_raw/1024.0, kd_raw/1024.0, ilim, dead, imin, imax
    def set_main_voltage_limits(self, min_v: float, max_v: float):
        # volts -> tenths
        mn = int(min_v*10)&0xFFFF; mx=int(max_v*10)&0xFFFF
        self._write_retry(self.address, CMD.SETMAINVOLTAGES, (mn>>8)&0xFF,mn&0xFF,(mx>>8)&0xFF,mx&0xFF)
        self._touch()
    def set_logic_voltage_limits(self, min_v: float, max_v: float):
        mn = int(min_v*10)&0xFFFF; mx=int(max_v*10)&0xFFFF
        self._write_retry(self.address, CMD.SETLOGICVOLTAGES, (mn>>8)&0xFF,mn&0xFF,(mx>>8)&0xFF,mx&0xFF)
        self._touch()

    def read_min_max_main_voltages(self) -> Tuple[float, float]:
        """Read configured main battery min/max voltage limits (volts)."""
        data = self._read_command(CMD.GETMINMAXMAINVOLTAGES, 4)
        mn, mx = struct.unpack('>HH', data)
        return mn/10.0, mx/10.0

    def read_min_max_logic_voltages(self) -> Tuple[float, float]:
        """Read configured logic battery min/max voltage limits (volts)."""
        data = self._read_command(CMD.GETMINMAXLOGICVOLTAGES, 4)
        mn, mx = struct.unpack('>HH', data)
        return mn/10.0, mx/10.0

    # Error decoding
    _ERROR_BITS = {
        0: 'E_STOP',
        1: 'TEMPERATURE',
        2: 'MAIN_BATTERY_HIGH',
        3: 'MAIN_BATTERY_LOW',
        4: 'LOGIC_BATTERY_HIGH',
        5: 'LOGIC_BATTERY_LOW',
        6: 'MOTOR1_DRIVE',
        7: 'MOTOR2_DRIVE',
        8: 'MOTOR1_OVERCURRENT',
        9: 'MOTOR2_OVERCURRENT'
    }
    def decode_errors(self, mask: int):
        return [name for bit,name in self._ERROR_BITS.items() if mask & (1<<bit)] or ['NONE']
    def snapshot(self) -> dict:
        """Batch read frequently displayed values (non-atomic)."""
        snap = {}
        try:
            m1e,_ = self.read_enc_m1(); m2e,_ = self.read_enc_m2()
            snap['enc1']=m1e; snap['enc2']=m2e
            m1s,_ = self.read_speed_m1(); m2s,_ = self.read_speed_m2()
            snap['speed1']=m1s; snap['speed2']=m2s
            c1,c2 = self.read_currents(); snap['current1']=c1; snap['current2']=c2
            snap['mbatt']=self.read_main_battery_voltage(); snap['lbatt']=self.read_logic_battery_voltage()
            snap['temp']=self.read_temperature()
            snap['errors']=self.read_error_status(); snap['error_list']=self.decode_errors(snap['errors'])
        except Exception as e:
            snap['snapshot_error']=str(e)
        return snap
    # Legacy compatibility wrappers
    def get_version(self):
        try: return self.read_version()
        except Exception: return None
    def get_main_battery_voltage(self):
        try: return self.read_main_battery_voltage()
        except Exception: return None
    def get_logic_battery_voltage(self):
        try: return self.read_logic_battery_voltage()
        except Exception: return None
    def get_temperature(self):
        try: return self.read_temperature()
        except Exception: return None
    def get_encoder_m1(self):
        try: return self.read_enc_m1()
        except Exception: return None
    def get_encoder_m2(self):
        try: return self.read_enc_m2()
        except Exception: return None
    def get_speed_m1(self):
        try: return self.read_speed_m1()
        except Exception: return None, None
    def get_speed_m2(self):
        try: return self.read_speed_m2()
        except Exception: return None, None
    def get_currents(self):
        try: return self.read_currents()
        except Exception: return None
    def get_error_status(self):
        try: return self.read_error_status()
        except Exception: return None

    def get_min_max_main_voltages(self):
        try: return self.read_min_max_main_voltages()
        except Exception: return None
    def get_min_max_logic_voltages(self):
        try: return self.read_min_max_logic_voltages()
        except Exception: return None

    # Legacy compatibility helper (renamed to avoid recursive redefinition bug)
    def legacy_reset_encoders(self):
        try:
            self.reset_encoders()
            return True
        except Exception:
            return False
    def stop_motors(self):
        try: self.stop_all(); return True
        except Exception: return False

    def get_stats(self):
        """Return protocol I/O statistics."""
        return dict(self._stats)
