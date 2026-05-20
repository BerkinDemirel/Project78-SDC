# kart_control.py
# ─────────────────────────────────────────────
# Single Arduino, single serial connection.
#
# Usage in sdc.py:
#   from kart_control import KartController, angle_to_fraction
#
#   kart = KartController(port="/dev/ttyUSB0")
#   kart.release()                          # must call before kart can move
#   kart.steer(angle_to_fraction(angle))    # -1.0 … +1.0
#   kart.brake(0.5)                         # 0.0 released … 1.0 full
#   kart.estop()                            # full brake + latch
#   kart.brake_reset()                      # resync open-loop estimate to 0
#   kart.close()
# ─────────────────────────────────────────────

import serial
import json
import threading
import time
import logging

log = logging.getLogger(__name__)

STEERING_ANGLE_LIMIT = 90.0

# Match STEER_POT_MIN / CTR / MAX in the .ino
POT_MIN = 452
POT_CTR = 606
POT_MAX = 743


def angle_to_fraction(angle_deg: float) -> float:
    """Convert steering angle degrees (-90 … +90) to -1.0 … +1.0."""
    return max(-1.0, min(1.0, angle_deg / STEERING_ANGLE_LIMIT))


def pot_to_fraction(raw: int) -> float:
    """Convert raw pot value to -1.0 … +1.0 using asymmetric centre."""
    if raw >= POT_CTR:
        return min(1.0,  (raw - POT_CTR) / (POT_MAX - POT_CTR))
    else:
        return max(-1.0, (raw - POT_CTR) / (POT_CTR - POT_MIN))


# ──────────────────────────────────────────────
# SHARED SERIAL CONNECTION
# ──────────────────────────────────────────────

class _KartSerial:
    """
    Manages the single serial connection to the Arduino.
    Handles connection, reconnection, background telemetry reader.
    """

    def __init__(self, port: str, baud: int = 115200,
                 timeout: float = 1.0, auto_reconnect: bool = True):
        self.port           = port
        self.baud           = baud
        self.timeout        = timeout
        self.auto_reconnect = auto_reconnect

        self._ser        = None
        self._write_lock = threading.Lock()
        self._telem_lock = threading.Lock()
        self._telemetry  = {}
        self._connected  = False
        self._stopped    = False

        self._connect()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _connect(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self._connected = True
            log.info(f"KartSerial connected on {self.port}")
        except serial.SerialException as e:
            self._connected = False
            log.error(f"KartSerial: cannot open {self.port}: {e}")

    def _reconnect(self):
        log.warning("KartSerial: reconnecting...")
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        time.sleep(2.0)
        self._connect()

    @property
    def connected(self) -> bool:
        return self._connected

    def _read_loop(self):
        while not self._stopped:
            if not self._connected:
                if self.auto_reconnect:
                    self._reconnect()
                else:
                    time.sleep(0.5)
                continue
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                data = json.loads(line)
                with self._telem_lock:
                    self._telemetry = data
            except json.JSONDecodeError:
                pass
            except serial.SerialException as e:
                log.warning(f"KartSerial read error: {e}")
                self._connected = False
            except Exception as e:
                log.warning(f"KartSerial error: {e}")

    def send(self, payload: dict) -> bool:
        if not self._connected or self._ser is None:
            log.warning(f"KartSerial: not connected, dropped: {payload}")
            return False
        try:
            line = json.dumps(payload, separators=(',', ':')) + '\n'
            with self._write_lock:
                self._ser.write(line.encode("utf-8"))
            return True
        except serial.SerialException as e:
            log.warning(f"KartSerial send error: {e}")
            self._connected = False
            return False

    @property
    def telemetry(self) -> dict:
        with self._telem_lock:
            return dict(self._telemetry)

    def close(self):
        self._stopped = True
        self._reader.join(timeout=3)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        log.info("KartSerial closed.")


# ──────────────────────────────────────────────
# KART CONTROLLER
# ──────────────────────────────────────────────

class KartController:
    """
    Single interface for steering + brake on one Arduino.

    Boot behaviour: Arduino boots with full brake applied.
    You MUST call release() before the kart can move.

    Args:
        port: serial port e.g. "/dev/ttyUSB0" or "COM3"

    Usage:
        kart = KartController(port="/dev/ttyUSB0")
        kart.release()                          # release brake before driving
        kart.steer(angle_to_fraction(angle))    # -1.0 … +1.0
        kart.brake(0.5)                         # 0.0 released … 1.0 full
        kart.estop()                            # full brake + latch
        kart.brake_reset()                      # resync estimate after manual release
        kart.close()
    """

    def __init__(self, port: str, baud: int = 115200, auto_reconnect: bool = True):
        self._serial = _KartSerial(port, baud, auto_reconnect=auto_reconnect)

    @property
    def connected(self) -> bool:
        return self._serial.connected

    # ── Steering ──────────────────────────────

    def steer(self, value: float):
        """Set steering. value: -1.0 (left) … +1.0 (right)."""
        value = max(-1.0, min(1.0, float(value)))
        self._serial.send({"cmd": "steer", "value": round(value, 4)})

    def centre(self):
        self.steer(0.0)

    def steer_enable(self, on: bool = True):
        self._serial.send({"cmd": "enable", "axis": "steer", "on": on})

    @property
    def steer_position(self) -> float:
        """Current steering position as -1.0 … +1.0 using asymmetric pot."""
        raw = self._serial.telemetry.get("sp", POT_CTR)
        return pot_to_fraction(raw)

    # ── Brake ─────────────────────────────────

    def brake(self, value: float):
        """Set brake fraction. value: 0.0 (released) … 1.0 (full)."""
        value = max(0.0, min(1.0, float(value)))
        self._serial.send({"cmd": "brake", "value": round(value, 4)})

    def release(self):
        """Fully release the brake. Call this before driving."""
        self._serial.send({"cmd": "release"})

    def brake_reset(self):
        """
        Resync the Arduino open-loop brake estimate to 0.
        Only call when you know the actuator is fully released.
        """
        self._serial.send({"cmd": "brake_reset"})

    # ── Shared ────────────────────────────────

    def estop(self):
        """Full brake immediately + latch. Requires release() to recover."""
        self._serial.send({"cmd": "estop"})

    @property
    def telemetry(self) -> dict:
        """Latest telemetry dict from Arduino: sp, st, bf, bt, es."""
        return self._serial.telemetry

    @property
    def is_estopped(self) -> bool:
        return bool(self._serial.telemetry.get("es", False))

    @property
    def brake_fraction(self) -> float:
        """Current brake fraction (open-loop estimate from Arduino)."""
        return float(self._serial.telemetry.get("bf", 0.0))

    def close(self):
        """Centre steering, apply brake, then close serial."""
        self.centre()
        time.sleep(0.3)
        self.estop()
        time.sleep(0.5)
        self._serial.close()