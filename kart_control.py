# kart_control.py
# ─────────────────────────────────────────────
# Single Arduino, single serial connection.
# SteeringController and BrakeController share
# one KartSerial instance under the hood.
#
# Usage in main.py (unchanged from before):
#   from kart_control import SteeringController, BrakeController, angle_to_fraction
#
#   steer = SteeringController(port="/dev/ttyUSB0")
#   brake = BrakeController(steer)      # shares the same connection
#
#   brake.release()                     # must call before kart can move
#   steer.steer(angle_to_fraction(steering_angle))
#   brake.brake(0.5)
#   steer.estop()   # or brake.estop() — both send {"cmd":"estop"}
#   steer.close()   # closes shared connection (brake closes too)
# ─────────────────────────────────────────────

import serial
import json
import threading
import time
import logging

log = logging.getLogger(__name__)

STEERING_ANGLE_LIMIT = 90.0


def angle_to_fraction(angle_deg: float) -> float:
    """Convert steering angle degrees (-90…+90) to -1.0…+1.0."""
    return max(-1.0, min(1.0, angle_deg / STEERING_ANGLE_LIMIT))


# ──────────────────────────────────────────────
# SHARED SERIAL CONNECTION
# ──────────────────────────────────────────────

class KartSerial:
    """
    Manages the single serial connection to the Arduino.
    Handles connection, reconnection, background telemetry reader.
    Shared between SteeringController and BrakeController.
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
# STEERING CONTROLLER
# ──────────────────────────────────────────────

class SteeringController:
    """
    Steering via BTS7960B with potentiometer feedback.
    Creates the shared KartSerial connection.

    Args:
        port: serial port e.g. "/dev/ttyUSB0" or "COM3"

    Usage:
        steer = SteeringController(port="/dev/ttyUSB0")
        steer.steer(0.35)       # -1.0 … +1.0
        steer.centre()
        steer.estop()
        steer.close()           # also closes shared serial
    """

    # Match these to STEER_POT_MIN / MAX in the .ino
    POT_MIN = 50
    POT_MAX = 973

    def __init__(self, port: str, baud: int = 115200, auto_reconnect: bool = True):
        self._serial = KartSerial(port, baud, auto_reconnect=auto_reconnect)

    # expose serial so BrakeController can share it
    @property
    def _conn(self):
        return self._serial

    @property
    def connected(self) -> bool:
        return self._serial.connected

    def steer(self, value: float):
        """Set steering. value: -1.0 (left) … +1.0 (right)."""
        value = max(-1.0, min(1.0, float(value)))
        self._serial.send({"cmd": "steer", "value": round(value, 4)})

    def centre(self):
        self.steer(0.0)

    def estop(self):
        self._serial.send({"cmd": "estop"})

    def enable(self, on: bool = True):
        self._serial.send({"cmd": "enable", "axis": "steer", "on": on})

    @property
    def position(self) -> float:
        """Current steering position as -1.0 … +1.0 (from pot feedback)."""
        raw  = self._serial.telemetry.get("sp", (self.POT_MIN + self.POT_MAX) // 2)
        mid  = (self.POT_MIN + self.POT_MAX) / 2.0
        half = (self.POT_MAX - self.POT_MIN) / 2.0
        return max(-1.0, min(1.0, (raw - mid) / half))

    @property
    def telemetry(self) -> dict:
        return self._serial.telemetry

    def close(self):
        """Centre steering then close shared serial connection."""
        self.centre()
        time.sleep(0.3)
        self._serial.close()


# ──────────────────────────────────────────────
# BRAKE CONTROLLER
# ──────────────────────────────────────────────

class BrakeController:
    """
    Brake via BTS7960B, open-loop timed (no pot).
    Shares the serial connection from SteeringController.

    Boot behaviour: Arduino boots with full brake applied.
    You MUST call release() before the kart can move.

    Args:
        steering: the SteeringController instance (shares its serial)

    Usage:
        brake = BrakeController(steer)
        brake.release()         # release before driving
        brake.brake(0.5)        # 0.0 released … 1.0 full
        brake.estop()           # full brake immediately
        # no need to call brake.close() — steer.close() handles it
    """

    PING_INTERVAL = 0.3   # seconds between automatic keepalives

    def __init__(self, steering: SteeringController):
        self._serial    = steering._conn   # shared connection
        self._stopped   = False
        self._last_ping = 0.0

        self._pinger = threading.Thread(target=self._ping_loop, daemon=True)
        self._pinger.start()

    def _ping_loop(self):
        """Keep the Arduino brake watchdog alive automatically."""
        while not self._stopped:
            now = time.time()
            if now - self._last_ping >= self.PING_INTERVAL:
                self._serial.send({"cmd": "ping"})
                self._last_ping = now
            time.sleep(0.05)

    def brake(self, value: float):
        """Set brake. value: 0.0 (released) … 1.0 (full brake)."""
        value = max(0.0, min(1.0, float(value)))
        self._serial.send({"cmd": "brake", "value": round(value, 4)})
        self._last_ping = time.time()

    def release(self):
        """Fully release the brake. Call this before driving."""
        self._serial.send({"cmd": "release"})
        self._last_ping = time.time()

    def estop(self):
        """Full brake immediately."""
        self._serial.send({"cmd": "estop"})
        self._last_ping = time.time()

    @property
    def fraction(self) -> float:
        """Last brake fraction reported by Arduino (open-loop estimate)."""
        return float(self._serial.telemetry.get("bf", 0.0))

    @property
    def is_estopped(self) -> bool:
        return bool(self._serial.telemetry.get("es", False))

    @property
    def telemetry(self) -> dict:
        return self._serial.telemetry

    def stop(self):
        """Stop pinger thread (called automatically when serial closes)."""
        self._stopped = True
        self._pinger.join(timeout=2)
