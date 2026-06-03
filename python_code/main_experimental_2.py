"""
main_experimental.py — SDC with traffic-sign brake control + stop-line detection
                        + improved lane detection + intersection navigation.

Stop sign  → slow to approach speed, brake AT the stop line, hold 10 s, resume.
Red light  → slow to approach speed, brake AT the stop line, wait for green, resume.

Keys:
  q  — quit
  e  — emergency stop
  o  — toggle YOLO object detection ON / OFF
  p  — pause / resume  (video mode only)
"""

from ultralytics import YOLO
import cv2
import numpy as np
import time
import threading
import math
import sys
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from tkinter import *
from typing import Optional, Tuple, List
from kart_control import KartController, angle_to_fraction

# ── Kart control ──────────────────────────────────────────────────────────────

KART_PORT    = "/dev/ttyUSB0"
KART_ENABLED = True

# ── YOLO ──────────────────────────────────────────────────────────────────────

model = YOLO('object_models/best.pt')

YOLO_ENABLED = False   # toggle with 'o' key at runtime

# ── Steering settings ─────────────────────────────────────────────────────────

STEERING_ANGLE_LIMIT = 90.0

# ── ROI trapezoid (fractions of frame size) ───────────────────────────────────

ROI_TOP_Y     = 0.3
ROI_BOTTOM_Y  = 0.95
ROI_TOP_LEFT  = 0.20
ROI_TOP_RIGHT = 0.80
ROI_BOT_LEFT  = 0
ROI_BOT_RIGHT = 1

# ── Lane detection tuning ─────────────────────────────────────────────────────

POLY_DEGREE            = 2
ALPHA_FRESH            = 0.35
ALPHA_MEDIUM           = 0.15
ALPHA_STALE            = 0.04
MIN_VERTICAL_SPAN_FRAC = 0.15
ASSUMED_LANE_WIDTH_FRAC= 0.6
MAX_HISTORY            = 7
N_POLY_POINTS          = 20
HOUGH_THRESHOLD        = 30
HOUGH_MIN_LEN          = 60
HOUGH_MAX_GAP          = 120
HOUGH_MIN_SLOPE        = 0.40

# ── Dot / road-stud filter ────────────────────────────────────────────────────

DOT_MIN_AREA = 5000

# ── X-zone lane side thresholds ──────────────────────────────────────────────

LEFT_X_MAX_FRAC  = 0.60
RIGHT_X_MIN_FRAC = 0.40

# ── Stop line detection ───────────────────────────────────────────────────────
# A painted stop / give-way line is a wide, near-horizontal stripe in the lower
# portion of the frame.  All three conditions must pass simultaneously.

STOP_LINE_MIN_SPAN   = 0.40   # must span at least 40% of frame width
STOP_LINE_MAX_SLOPE  = 0.20   # near-horizontal: |dy/dx| < this value
STOP_LINE_MIN_Y_FRAC = 0.55   # must be in the lower 45% of the frame (close)

# ── Intersection detection (heuristic, no extra training needed) ──────────────

INTERSECTION_LINE_BURST = 12   # raw Hough count above this = line explosion

# ── Steering display ──────────────────────────────────────────────────────────

TURN_THRESHOLD_DEG   = 4.0
STEERING_LINE_Y_FRAC = 0.82

# ── Traffic control ───────────────────────────────────────────────────────────

STOP_LABELS          = {"stop-sign"}
RED_LABELS           = {"red"}
GREEN_LABELS         = {"green"}
STOP_HOLD_DURATION   = 10.0
BRAKE_RAMP_DURATION  = 1.0
DETECTION_CONFIDENCE = 0.5
APPROACH_SPEED       = 0.35   # fraction of full speed while creeping to stop line
APPROACH_TIMEOUT     = 4.0    # seconds — brake hard even if no line found in time

# ── Misc ──────────────────────────────────────────────────────────────────────

PRINT_EVERY = 10
frame_count  = 0


# ══════════════════════════════════════════════════════════════════════════════
# DATA CLASSES
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class LaneState:
    left_poly:      Optional[np.ndarray] = None
    right_poly:     Optional[np.ndarray] = None
    left_conf:      float = 0.0
    right_conf:     float = 0.0
    error_hist:     List[float] = field(default_factory=list)
    left_real:      bool = False   # True = detected this frame, False = estimated
    right_real:     bool = False
    last_known_mid: Optional[int] = None


@dataclass
class IntersectionSignals:
    """Four independent heuristic signals.  Two or more = intersection confirmed."""
    horizontal_line: bool = False
    conf_collapse:   bool = False
    line_burst:      bool = False
    width_anomaly:   bool = False

    @property
    def score(self) -> int:
        return sum([self.horizontal_line, self.conf_collapse,
                    self.line_burst,      self.width_anomaly])

    @property
    def detected(self) -> bool:
        return self.score >= 2


# ══════════════════════════════════════════════════════════════════════════════
# TRAFFIC STATE MACHINE
# ══════════════════════════════════════════════════════════════════════════════

class DriveState(Enum):
    DRIVING          = "DRIVING"
    APPROACHING_LINE = "APPROACHING LINE"   # sign/light seen — slow & watch for line
    BRAKING          = "BRAKING"
    STOPPED_SIGN     = "STOPPED (stop sign)"
    STOPPED_RED      = "STOPPED (red light)"
    RESUMING         = "RESUMING"


class TrafficStateController:
    def __init__(self):
        self.state        = DriveState.DRIVING
        self.state_start  = time.time()
        self._brake_cause = None

    # ── helpers ───────────────────────────────────────────────────────────────

    def _best_match(self, detections, label_set):
        matches = [d for d in detections
                   if d['label'].lower() in label_set
                   and d['confidence'] >= DETECTION_CONFIDENCE]
        return max(matches, key=lambda d: d['confidence']) if matches else None

    def _transition(self, new_state, kart, reason=""):
        tag = f" ({reason})" if reason else ""
        print(f"[TRAFFIC] {self.state.value} → {new_state.value}{tag}")
        self.state       = new_state
        self.state_start = time.time()

        if new_state == DriveState.APPROACHING_LINE:
            # Slow down but do NOT brake yet
            if kart:
                kart.set_speed(APPROACH_SPEED)

        elif new_state == DriveState.BRAKING:
            if kart:
                kart.brake(1.0)

        elif new_state in (DriveState.STOPPED_SIGN, DriveState.STOPPED_RED):
            if kart:
                kart.brake(1.0)

        elif new_state == DriveState.RESUMING:
            if kart:
                kart.release()

    # ── main update ───────────────────────────────────────────────────────────

    def update(self, detections, kart, stop_line_detected: bool = False):
        """
        Call once per frame.

        stop_line_detected  — True when _detect_stop_line() fires AND we are
                              currently in APPROACHING_LINE state.  The caller
                              should gate this flag accordingly.

        Returns: (brake_active, state_label, status_text)
        """
        now     = time.time()
        elapsed = now - self.state_start
        stop_det  = self._best_match(detections, STOP_LABELS)
        red_det   = self._best_match(detections, RED_LABELS)
        green_det = self._best_match(detections, GREEN_LABELS)

        # ── state transitions ─────────────────────────────────────────────────

        if self.state == DriveState.DRIVING:
            if stop_det:
                self._brake_cause = "sign"
                self._transition(DriveState.APPROACHING_LINE, kart,
                                 reason=f"stop sign {stop_det['confidence']:.2f}")
            elif red_det:
                self._brake_cause = "red"
                self._transition(DriveState.APPROACHING_LINE, kart,
                                 reason=f"red light {red_det['confidence']:.2f}")

        elif self.state == DriveState.APPROACHING_LINE:
            # Brake when we actually reach the painted line — or time out.
            if stop_line_detected:
                self._transition(DriveState.BRAKING, kart, reason="stop line reached")
            elif elapsed >= APPROACH_TIMEOUT:
                self._transition(DriveState.BRAKING, kart, reason="approach timeout")

        elif self.state == DriveState.BRAKING:
            if elapsed >= BRAKE_RAMP_DURATION:
                if self._brake_cause == "sign":
                    self._transition(DriveState.STOPPED_SIGN, kart)
                else:
                    self._transition(DriveState.STOPPED_RED, kart)

        elif self.state == DriveState.STOPPED_SIGN:
            if elapsed >= STOP_HOLD_DURATION:
                self._transition(DriveState.RESUMING, kart, reason="10 s elapsed")

        elif self.state == DriveState.STOPPED_RED:
            if green_det:
                self._transition(DriveState.RESUMING, kart,
                                 reason=f"green light {green_det['confidence']:.2f}")

        elif self.state == DriveState.RESUMING:
            if elapsed >= 0.5:
                self._transition(DriveState.DRIVING, kart)

        # ── outputs ───────────────────────────────────────────────────────────

        brake_active = self.state in (
            DriveState.BRAKING,
            DriveState.STOPPED_SIGN,
            DriveState.STOPPED_RED,
        )
        return brake_active, self.state.value, self._status_text(elapsed)

    def _status_text(self, elapsed):
        if self.state == DriveState.APPROACHING_LINE:
            return "APPROACHING — watching for stop line"
        if self.state == DriveState.STOPPED_SIGN:
            return f"STOP SIGN — resuming in {max(0.0, STOP_HOLD_DURATION - elapsed):.1f}s"
        if self.state == DriveState.STOPPED_RED:
            return f"RED LIGHT — waiting {elapsed:.0f}s (need green)"
        if self.state == DriveState.BRAKING:
            return "BRAKING"
        if self.state == DriveState.RESUMING:
            return "RESUMING"
        return "DRIVING"


# ══════════════════════════════════════════════════════════════════════════════
# FRAME CAPTURE THREAD
# ══════════════════════════════════════════════════════════════════════════════

class FrameCapture:
    def __init__(self, source=0, max_retries=5):
        self.source      = source
        self.max_retries = max_retries
        self.cap         = self._open_cap()
        self.frame       = None
        self.lock        = threading.Lock()
        self.stopped     = False
        self.thread      = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _open_cap(self):
        cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera: {self.source}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS,          30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        return cap

    def _reader(self):
        retries = 0
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                retries += 1
                print(f"Failed to grab frame ({retries}/{self.max_retries})")
                if retries >= self.max_retries:
                    print("Max retries — attempting reconnect...")
                    self.cap.release()
                    time.sleep(1.0)
                    try:
                        self.cap = self._open_cap()
                        retries  = 0
                    except RuntimeError as e:
                        print(f"Reconnect failed: {e}")
                        self.stopped = True
                        break
                else:
                    time.sleep(0.1)
                continue
            retries = 0
            with self.lock:
                self.frame = frame

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def release(self):
        self.stopped = True
        self.thread.join(timeout=3)
        self.cap.release()


# ══════════════════════════════════════════════════════════════════════════════
# LANE DETECTION — helpers
# ══════════════════════════════════════════════════════════════════════════════

def _roi_mask(shape):
    """Trapezoid ROI — narrow at top (horizon), wide at bottom (near kart)."""
    h, w = shape
    mask = np.zeros((h, w), dtype=np.uint8)
    pts  = np.array([[
        (int(w * ROI_BOT_LEFT),  int(h * ROI_BOTTOM_Y)),
        (int(w * ROI_TOP_LEFT),  int(h * ROI_TOP_Y)),
        (int(w * ROI_TOP_RIGHT), int(h * ROI_TOP_Y)),
        (int(w * ROI_BOT_RIGHT), int(h * ROI_BOTTOM_Y)),
    ]], dtype=np.int32)
    cv2.fillPoly(mask, pts, 255)
    return mask


def _remove_small_blobs(mask, min_area=DOT_MIN_AREA):
    """Remove connected white regions smaller than min_area pixels (road studs)."""
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask, connectivity=8)
    clean = np.zeros_like(mask)
    for lbl in range(1, num_labels):
        if stats[lbl, cv2.CC_STAT_AREA] >= min_area:
            clean[labels == lbl] = 255
    return clean


def _edge_image(frame):
    """
    Edge image highlighting white/yellow lane markings, suppressing road studs.

    Pipeline:
      1. HLS colour mask for white and yellow.
      2. Morphological opening on white mask — kills narrow dots.
      3. Connected-component area filter — drops remaining small blobs.
      4. CLAHE + Gaussian blur + Canny on the L channel.
      5. AND edges with colour mask.
    """
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    white  = cv2.inRange(hls, np.array([0,   130,   0]),
                              np.array([255, 255,  60]))
    yellow = cv2.inRange(hls, np.array([15,   80,  80]),
                              np.array([35,  255, 255]))

    dot_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    white_clean = cv2.morphologyEx(white, cv2.MORPH_OPEN, dot_kernel, iterations=1)
    white_clean = _remove_small_blobs(white_clean, DOT_MIN_AREA)
    color_mask  = cv2.bitwise_or(white_clean, yellow)

    l, a, b = cv2.split(cv2.cvtColor(frame, cv2.COLOR_BGR2LAB))
    cl      = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(l)
    blur    = cv2.GaussianBlur(cl, (7, 7), 0)
    edges   = cv2.Canny(blur, 50, 150)

    return cv2.bitwise_and(edges, color_mask)


def _fit_poly(pts_x, pts_y, h):
    if len(pts_y) < 4:
        return None, 0.0
    pts_x = np.array(pts_x, dtype=float)
    pts_y = np.array(pts_y, dtype=float)
    span  = (pts_y.max() - pts_y.min()) / h
    if span < MIN_VERTICAL_SPAN_FRAC:
        return None, 0.0
    try:
        coeffs = np.polyfit(pts_y, pts_x, POLY_DEGREE)
    except np.linalg.LinAlgError:
        return None, 0.0
    return coeffs, min(1.0, span / 0.5)


def _sample_poly(coeffs, y_top, y_bot, n=N_POLY_POINTS):
    ys = np.linspace(y_bot, y_top, n)
    xs = np.polyval(coeffs, ys)
    return [(int(x), int(y)) for x, y in zip(xs, ys)]


def _smooth_poly(current, previous, conf):
    if current is None:
        if previous is None:
            return None, 0.0
        new_conf = conf * (1 - ALPHA_STALE)
        return (None, 0.0) if new_conf < 0.05 else (previous, new_conf)
    if previous is None:
        return current, conf
    alpha   = ALPHA_FRESH if conf > 0.7 else ALPHA_MEDIUM
    blended = alpha * current + (1 - alpha) * previous
    return blended, alpha * conf + (1 - alpha) * 1.0


# ── Stop line detection ───────────────────────────────────────────────────────

def _detect_stop_line(lines, frame_shape) -> bool:
    """
    Returns True when a wide, near-horizontal painted line appears in the lower
    portion of the frame — the stop / give-way line at an intersection.

    Operates on the RAW HoughLinesP output before the lane-line slope filter
    discards horizontal segments.

    Tune:
      STOP_LINE_MIN_SPAN    raise to reduce false positives (shadows, crossings)
      STOP_LINE_MAX_SLOPE   lower to require a more horizontal line
      STOP_LINE_MIN_Y_FRAC  raise to require the line to be closer to the kart
    """
    if lines is None:
        return False
    h, w   = frame_shape[:2]
    min_y  = h * STOP_LINE_MIN_Y_FRAC
    for ln in lines:
        x1, y1, x2, y2 = ln[0]
        if min(y1, y2) < min_y:
            continue                          # line is too far away
        if x2 == x1:
            continue
        slope = abs((y2 - y1) / (x2 - x1))
        span  = abs(x2 - x1) / w
        if slope < STOP_LINE_MAX_SLOPE and span > STOP_LINE_MIN_SPAN:
            return True
    return False


# ── Intersection heuristics ───────────────────────────────────────────────────

def _detect_horizontal_lines(lines, w) -> bool:
    """Wide, near-horizontal Hough lines = likely cross-road marking."""
    if lines is None:
        return False
    for ln in lines:
        x1, y1, x2, y2 = ln[0]
        if x2 == x1:
            continue
        slope = abs((y2 - y1) / (x2 - x1))
        span  = abs(x2 - x1) / w
        if slope < 0.25 and span > 0.30:
            return True
    return False


def _lanes_collapsing(state: LaneState) -> bool:
    """Both lane confidences low while history shows a previously stable lane."""
    both_weak = state.left_conf < 0.35 and state.right_conf < 0.35
    had_lane  = len(state.error_hist) >= 3
    return both_weak and had_lane


def _lane_width_anomaly(left_pts, right_pts, w,
                        expected_frac=ASSUMED_LANE_WIDTH_FRAC,
                        tolerance=0.25) -> bool:
    """Apparent lane width jumps when cross-road geometry enters the ROI."""
    if not left_pts or not right_pts:
        return False
    actual = abs(right_pts[0][0] - left_pts[0][0]) / w
    return abs(actual - expected_frac) > tolerance


def _build_intersection_signals(lines, left_pts, right_pts,
                                 state: LaneState, w) -> IntersectionSignals:
    line_count = len(lines) if lines is not None else 0
    return IntersectionSignals(
        horizontal_line = _detect_horizontal_lines(lines, w),
        conf_collapse   = _lanes_collapsing(state),
        line_burst      = line_count >= INTERSECTION_LINE_BURST,
        width_anomaly   = _lane_width_anomaly(left_pts, right_pts, w),
    )


# ── Persistence filter for intersection detection ─────────────────────────────

class IntersectionDetector:
    """
    Requires N consecutive frames with 2+ heuristic signals before confirming.
    After confirmation, suppresses re-detection for cooldown_frames frames so
    the kart doesn't re-trigger mid-turn.
    """
    def __init__(self, confirm_frames=4, cooldown_frames=60):
        self._buffer         = deque(maxlen=confirm_frames)
        self._cooldown       = 0
        self.confirm_frames  = confirm_frames
        self.cooldown_frames = cooldown_frames

    def update(self, signals: IntersectionSignals) -> bool:
        if self._cooldown > 0:
            self._cooldown -= 1
            return False
        self._buffer.append(signals.detected)
        confirmed = sum(self._buffer) >= self.confirm_frames
        if confirmed:
            self._buffer.clear()
            self._cooldown = self.cooldown_frames
            return True
        return False


# ── Core lane detection ───────────────────────────────────────────────────────

def _detect_lines_worker(frame, state: LaneState):
    """
    Returns:
        left_pts           — list of (x,y) points for the left lane line (or None)
        right_pts          — list of (x,y) points for the right lane line (or None)
        state              — updated LaneState
        stop_line_detected — bool: wide horizontal line in lower frame
        intersection_sigs  — IntersectionSignals dataclass
    """
    h, w    = frame.shape[:2]
    y_top   = int(h * ROI_TOP_Y)
    y_bot   = int(h * ROI_BOTTOM_Y)
    edges   = _edge_image(frame)
    mask    = _roi_mask((h, w))
    cropped = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(cropped, 1, np.pi / 180,
                            threshold=HOUGH_THRESHOLD,
                            minLineLength=HOUGH_MIN_LEN,
                            maxLineGap=HOUGH_MAX_GAP)

    # ── Stop line — check raw lines BEFORE slope filter ──────────────────────
    stop_line_detected = _detect_stop_line(lines, frame.shape)

    # ── Lane line extraction ──────────────────────────────────────────────────
    left_xs, left_ys, right_xs, right_ys = [], [], [], []
    left_x_max  = w * LEFT_X_MAX_FRAC
    right_x_min = w * RIGHT_X_MIN_FRAC

    if lines is not None:
        for ln in lines:
            x1, y1, x2, y2 = ln[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < HOUGH_MIN_SLOPE:
                continue
            mid_x = (x1 + x2) / 2
            if slope < 0 and mid_x < left_x_max:
                left_xs  += [x1, x2];  left_ys  += [y1, y2]
            elif slope > 0 and mid_x > right_x_min:
                right_xs += [x1, x2];  right_ys += [y1, y2]

    new_left,  lc = _fit_poly(left_xs,  left_ys,  h)
    new_right, rc = _fit_poly(right_xs, right_ys, h)
    state.left_poly,  state.left_conf  = _smooth_poly(new_left,  state.left_poly,  lc)
    state.right_poly, state.right_conf = _smooth_poly(new_right, state.right_poly, rc)
    state.left_real  = new_left  is not None
    state.right_real = new_right is not None

    left_pts  = _sample_poly(state.left_poly,  y_top, y_bot) if state.left_poly  is not None else None
    right_pts = _sample_poly(state.right_poly, y_top, y_bot) if state.right_poly is not None else None

    # Estimated (visual only) missing lane line
    lane_w = int(ASSUMED_LANE_WIDTH_FRAC * w)
    if left_pts is not None and right_pts is None:
        right_pts = [(x + lane_w, y) for x, y in left_pts]
    elif right_pts is not None and left_pts is None:
        left_pts  = [(x - lane_w, y) for x, y in right_pts]

    # ── Intersection heuristics ───────────────────────────────────────────────
    intersection_sigs = _build_intersection_signals(lines, left_pts, right_pts, state, w)

    return left_pts, right_pts, state, stop_line_detected, intersection_sigs


def _compute_steering(frame_width, left_pts, right_pts, state: LaneState):
    cx = frame_width // 2
    lx = left_pts[0][0]  if left_pts  else None
    rx = right_pts[0][0] if right_pts else None

    if state.left_real and state.right_real:
        lane_center          = (lx + rx) // 2
        state.last_known_mid = lane_center
    elif state.left_real and state.last_known_mid is not None:
        dist        = state.last_known_mid - lx
        lane_center = lx + dist
    elif state.right_real and state.last_known_mid is not None:
        dist        = rx - state.last_known_mid
        lane_center = rx - dist
    elif state.left_real:
        lane_center = lx + int(ASSUMED_LANE_WIDTH_FRAC * frame_width / 2)
    elif state.right_real:
        lane_center = rx - int(ASSUMED_LANE_WIDTH_FRAC * frame_width / 2)
    else:
        if state.last_known_mid is not None:
            lane_center = state.last_known_mid
        else:
            last = state.error_hist[-1] if state.error_hist else 0.0
            state.error_hist.append(last)
            if len(state.error_hist) > MAX_HISTORY:
                state.error_hist.pop(0)
            smoothed = np.mean(state.error_hist)
            return float(np.clip(smoothed * STEERING_ANGLE_LIMIT,
                                 -STEERING_ANGLE_LIMIT, STEERING_ANGLE_LIMIT))

    error      = lane_center - cx
    normalized = float(np.clip(error / (frame_width * 0.30), -1.0, 1.0))
    if abs(normalized) < 0.03:
        normalized = 0.0
    state.error_hist.append(normalized)
    if len(state.error_hist) > MAX_HISTORY:
        state.error_hist.pop(0)
    smoothed = float(np.mean(state.error_hist))
    return float(np.clip(smoothed * STEERING_ANGLE_LIMIT,
                         -STEERING_ANGLE_LIMIT, STEERING_ANGLE_LIMIT))


# ══════════════════════════════════════════════════════════════════════════════
# LANE DETECTION THREAD
# ══════════════════════════════════════════════════════════════════════════════

class LaneDetector:
    def __init__(self):
        self.input_frame       = None
        self.left_pts          = None
        self.right_pts         = None
        self.steering_angle    = 0.0
        self.stop_line         = False
        self.intersection_sigs = IntersectionSignals()
        self._lane_state       = LaneState()
        self.input_lock        = threading.Lock()
        self.output_lock       = threading.Lock()
        self.stopped           = False
        self.thread            = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, frame):
        with self.input_lock:
            self.input_frame = frame.copy()

    def _run(self):
        while not self.stopped:
            with self.input_lock:
                frame            = self.input_frame
                self.input_frame = None
            if frame is None:
                time.sleep(0.001)
                continue

            left_pts, right_pts, self._lane_state, stop_line, isect_sigs = \
                _detect_lines_worker(frame, self._lane_state)

            angle = _compute_steering(frame.shape[1], left_pts, right_pts,
                                      self._lane_state)

            with self.output_lock:
                self.left_pts          = left_pts
                self.right_pts         = right_pts
                self.steering_angle    = angle
                self.stop_line         = stop_line
                self.intersection_sigs = isect_sigs

    def read(self):
        with self.output_lock:
            return (self.left_pts, self.right_pts, self.steering_angle,
                    self._lane_state, self.stop_line, self.intersection_sigs)

    def release(self):
        self.stopped = True
        self.thread.join(timeout=3)


# ══════════════════════════════════════════════════════════════════════════════
# YOLO DETECTION THREAD
# ══════════════════════════════════════════════════════════════════════════════

class YoloDetector:
    def __init__(self):
        self.input_frame = None
        self.detections  = []
        self.boxes_frame = None
        self.input_lock  = threading.Lock()
        self.output_lock = threading.Lock()
        self.stopped     = False
        self.thread      = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, frame):
        with self.input_lock:
            self.input_frame = frame.copy()

    def _run(self):
        while not self.stopped:
            with self.input_lock:
                frame            = self.input_frame
                self.input_frame = None
            if frame is None:
                time.sleep(0.001)
                continue

            if not YOLO_ENABLED:
                with self.output_lock:
                    self.detections  = []
                    self.boxes_frame = None
                continue

            results     = model.predict(source=frame, conf=0.5, stream=False,
                                        half=True, verbose=False)
            boxes_frame = results[0].plot()
            detections  = []
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                label  = model.names[cls_id]
                conf   = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({"label": label, "confidence": conf,
                                   "bbox": (x1, y1, x2, y2)})
            with self.output_lock:
                self.detections  = detections
                self.boxes_frame = boxes_frame

    def read(self):
        with self.output_lock:
            boxes = self.boxes_frame.copy() if self.boxes_frame is not None else None
            return boxes, list(self.detections)

    def release(self):
        self.stopped = True
        self.thread.join(timeout=3)


# ══════════════════════════════════════════════════════════════════════════════
# STEERING WHEEL WIDGET
# ══════════════════════════════════════════════════════════════════════════════

def draw_steering_wheel(canvas, angle, width=300, height=300):
    canvas.delete("all")
    cx, cy = width // 2, height // 2
    radius = min(cx, cy) - 20
    canvas.create_oval(cx - radius, cy - radius,
                       cx + radius, cy + radius,
                       outline="#444", width=18, fill="#222")
    hub = radius * 0.18
    canvas.create_oval(cx - hub, cy - hub, cx + hub, cy + hub,
                       fill="#555", outline="#888", width=2)
    for spoke_offset in [0, 120, 240]:
        rad = math.radians(angle + spoke_offset)
        sx  = cx + radius * 0.85 * math.sin(rad)
        sy  = cy - radius * 0.85 * math.cos(rad)
        canvas.create_line(cx, cy, sx, sy, width=8, fill="#888")
    marker_rad = math.radians(angle)
    mx = cx + (radius - 8) * math.sin(marker_rad)
    my = cy - (radius - 8) * math.cos(marker_rad)
    canvas.create_oval(mx - 7, my - 7, mx + 7, my + 7, fill="red", outline="")
    direction = "RIGHT" if angle > 0.5 else "LEFT" if angle < -0.5 else "STRAIGHT"
    canvas.create_text(cx, cy + radius + 16,
                       text=f"{angle:.1f}°  {direction}",
                       fill="white", font=("Arial", 13, "bold"))


# ══════════════════════════════════════════════════════════════════════════════
# DRAW LANES
# ══════════════════════════════════════════════════════════════════════════════

def draw_lanes(frame, left_pts, right_pts, state: LaneState,
               steering_angle: float = 0.0,
               stop_line_active: bool = False,
               intersection_sigs: Optional[IntersectionSignals] = None):
    h, w    = frame.shape[:2]
    out     = frame.copy()
    overlay = np.zeros_like(out)

    def draw_polyline(pts, color, thickness=6):
        for i in range(len(pts) - 1):
            cv2.line(overlay, pts[i], pts[i + 1], color, thickness, cv2.LINE_AA)

    if left_pts and right_pts:
        poly = np.array(left_pts + right_pts[::-1], dtype=np.int32)
        cv2.fillPoly(overlay, [poly], (0, 80, 0))

    if left_pts:
        if state.left_real:
            draw_polyline(left_pts, (0, 220, 0), thickness=6)
        else:
            for i in range(0, len(left_pts) - 1, 2):
                cv2.line(overlay, left_pts[i], left_pts[i + 1],
                         (120, 120, 80), 4, cv2.LINE_AA)
    if right_pts:
        if state.right_real:
            draw_polyline(right_pts, (0, 220, 0), thickness=6)
        else:
            for i in range(0, len(right_pts) - 1, 2):
                cv2.line(overlay, right_pts[i], right_pts[i + 1],
                         (120, 120, 80), 4, cv2.LINE_AA)

    out = cv2.addWeighted(out, 1.0, overlay, 0.55, 0)

    # ── ROI trapezoid outline ─────────────────────────────────────────────────
    roi_pts = np.array([
        (int(w * ROI_BOT_LEFT),  int(h * ROI_BOTTOM_Y)),
        (int(w * ROI_TOP_LEFT),  int(h * ROI_TOP_Y)),
        (int(w * ROI_TOP_RIGHT), int(h * ROI_TOP_Y)),
        (int(w * ROI_BOT_RIGHT), int(h * ROI_BOTTOM_Y)),
    ], dtype=np.int32)
    roi_color = (255, 220, 0)
    dash, gap = 14, 8
    for p1, p2 in zip(roi_pts, np.roll(roi_pts, -1, axis=0)):
        x1, y1 = int(p1[0]), int(p1[1])
        x2, y2 = int(p2[0]), int(p2[1])
        seg    = max(1, int(np.hypot(x2 - x1, y2 - y1)))
        for s in range(seg // (dash + gap) + 1):
            t0 = s * (dash + gap) / seg
            t1 = min(1.0, t0 + dash / seg)
            cv2.line(out,
                     (int(x1 + t0*(x2-x1)), int(y1 + t0*(y2-y1))),
                     (int(x1 + t1*(x2-x1)), int(y1 + t1*(y2-y1))),
                     roi_color, 1, cv2.LINE_AA)
    for pt in roi_pts:
        cv2.circle(out, tuple(pt), 4, roi_color, -1, cv2.LINE_AA)
    cv2.putText(out, "ROI",
                (int(w * (ROI_TOP_LEFT + ROI_TOP_RIGHT) / 2) - 16,
                 int(h * ROI_TOP_Y) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi_color, 1, cv2.LINE_AA)

    # ── Stop line indicator ───────────────────────────────────────────────────
    if stop_line_active:
        stop_y = int(h * STOP_LINE_MIN_Y_FRAC)
        cv2.line(out, (0, stop_y), (w, stop_y), (0, 0, 255), 3, cv2.LINE_AA)
        cv2.putText(out, "STOP LINE", (w // 2 - 60, stop_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

    # ── Intersection signal indicator ─────────────────────────────────────────
    if intersection_sigs is not None and intersection_sigs.score >= 1:
        sig_text = (f"ISECT signals {intersection_sigs.score}/4 "
                    f"[H:{int(intersection_sigs.horizontal_line)} "
                    f"C:{int(intersection_sigs.conf_collapse)} "
                    f"B:{int(intersection_sigs.line_burst)} "
                    f"W:{int(intersection_sigs.width_anomaly)}]")
        color = (0, 165, 255) if intersection_sigs.detected else (180, 180, 60)
        cv2.putText(out, sig_text, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # ── Steering decision line ────────────────────────────────────────────────
    sl_y = int(h * STEERING_LINE_Y_FRAC)
    cx   = w // 2

    def _x_at_y(pts, target_y):
        if not pts:
            return None
        for i in range(len(pts) - 1):
            y_lo, y_hi = pts[i][1], pts[i + 1][1]
            if min(y_lo, y_hi) <= target_y <= max(y_lo, y_hi):
                if y_lo == y_hi:
                    return pts[i][0]
                t = (target_y - y_lo) / (y_hi - y_lo)
                return int(pts[i][0] + t * (pts[i + 1][0] - pts[i][0]))
        return None

    lx_sl = _x_at_y(left_pts,  sl_y) if state.left_real  else None
    rx_sl = _x_at_y(right_pts, sl_y) if state.right_real else None

    if lx_sl is not None and rx_sl is not None:
        lane_cx = (lx_sl + rx_sl) // 2
    elif lx_sl is not None and state.last_known_mid is not None:
        lane_cx = lx_sl + (state.last_known_mid - lx_sl)
    elif rx_sl is not None and state.last_known_mid is not None:
        lane_cx = rx_sl - (rx_sl - state.last_known_mid)
    elif lx_sl is not None:
        lane_cx = lx_sl + int(ASSUMED_LANE_WIDTH_FRAC * w / 2)
    elif rx_sl is not None:
        lane_cx = rx_sl - int(ASSUMED_LANE_WIDTH_FRAC * w / 2)
    elif state.last_known_mid is not None:
        lane_cx = state.last_known_mid
    else:
        lane_cx = cx

    abs_angle = abs(steering_angle)
    if abs_angle < TURN_THRESHOLD_DEG:
        arrow_col  = (0, 220, 0);   turn_label = "STRAIGHT"
    elif abs_angle < 25:
        arrow_col  = (0, 200, 255); turn_label = "RIGHT" if steering_angle > 0 else "LEFT"
    else:
        arrow_col  = (0, 60, 230);  turn_label = "SHARP RIGHT" if steering_angle > 0 else "SHARP LEFT"

    cv2.line(out, (0, sl_y), (w, sl_y), (60, 60, 60), 1, cv2.LINE_AA)
    cv2.line(out, (cx, sl_y - 18), (cx, sl_y + 18), (200, 200, 200), 2, cv2.LINE_AA)
    cv2.arrowedLine(out, (cx, sl_y), (lane_cx, sl_y),
                    arrow_col, 3, cv2.LINE_AA, tipLength=0.18)
    cv2.circle(out, (lane_cx, sl_y), 7, arrow_col, -1, cv2.LINE_AA)
    cv2.circle(out, (lane_cx, sl_y), 7, (255, 255, 255), 1, cv2.LINE_AA)
    lbl_x = lane_cx + (12 if lane_cx >= cx else -12)
    cv2.putText(out, turn_label, (lbl_x, sl_y - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, arrow_col, 2, cv2.LINE_AA)
    cv2.putText(out, f"offset {lane_cx - cx:+d}px",
                (cx - 55, sl_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
    return out


# ══════════════════════════════════════════════════════════════════════════════
# COMPOSITE FRAME BUILDER
# ══════════════════════════════════════════════════════════════════════════════

_STATE_COLOURS = {
    DriveState.DRIVING:          (0,   200,   0),
    DriveState.APPROACHING_LINE: (0,   200, 255),
    DriveState.BRAKING:          (0,   165, 255),
    DriveState.STOPPED_SIGN:     (0,     0, 220),
    DriveState.STOPPED_RED:      (0,     0, 220),
    DriveState.RESUMING:         (255, 200,   0),
}


def build_display(base_frame, boxes_frame, left_pts, right_pts,
                  lane_state, steering_angle,
                  fps_window, prev_time, font,
                  traffic_state=DriveState.DRIVING, traffic_text="DRIVING",
                  stop_line_active=False,
                  intersection_sigs=None):

    display = boxes_frame.copy() if boxes_frame is not None else base_frame.copy()
    display = draw_lanes(display, left_pts, right_pts, lane_state, steering_angle,
                         stop_line_active=stop_line_active,
                         intersection_sigs=intersection_sigs)

    direction = "RIGHT" if steering_angle > 0 else "LEFT" if steering_angle < 0 else "STRAIGHT"
    cv2.putText(display, f"Steer: {steering_angle:.1f}deg ({direction})",
                (10, 40), font, 1.0, (0, 255, 255), 2, cv2.LINE_AA)

    current_time = time.time()
    fps_window.append(1 / (current_time - prev_time))
    avg_fps = sum(fps_window) / len(fps_window)
    cv2.putText(display, f"FPS: {avg_fps:.1f}",
                (10, 80), font, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.putText(display,
                f"L:{lane_state.left_conf:.2f}  R:{lane_state.right_conf:.2f}",
                (10, 115), font, 0.6, (180, 180, 180), 1, cv2.LINE_AA)

    colour = _STATE_COLOURS.get(traffic_state, (200, 200, 200))
    cv2.rectangle(display, (0, 130), (620, 162), (30, 30, 30), -1)
    cv2.putText(display, f"Traffic: {traffic_text}",
                (10, 154), font, 0.9, colour, 2, cv2.LINE_AA)

    if not YOLO_ENABLED:
        bw, bh = 130, 28
        bx = display.shape[1] - bw - 10
        cv2.rectangle(display, (bx, 10), (bx + bw, 10 + bh), (0, 0, 200), -1)
        cv2.rectangle(display, (bx, 10), (bx + bw, 10 + bh), (0, 0, 100),  1)
        cv2.putText(display, "YOLO OFF", (bx + 14, 10 + 20),
                    font, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

    return display, direction, current_time


# ══════════════════════════════════════════════════════════════════════════════
# KART HELPER
# ══════════════════════════════════════════════════════════════════════════════

def make_kart():
    if not KART_ENABLED:
        return None
    kart = KartController(port=KART_PORT)
    if not kart.connected:
        print("WARNING: Arduino not connected — running without hardware output.")
        return None
    return kart


# ══════════════════════════════════════════════════════════════════════════════
# CAMERA MODE
# ══════════════════════════════════════════════════════════════════════════════

def run_on_camera(source=1):
    global frame_count, YOLO_ENABLED
    frame_count = 0

    kart = make_kart()
    if kart:
        kart.release()

    capture      = FrameCapture(source=source, max_retries=5)
    lane_det     = LaneDetector()
    yolo_det     = YoloDetector()
    traffic      = TrafficStateController()
    isect_filter = IntersectionDetector(confirm_frames=4, cooldown_frames=60)

    fps_window = deque(maxlen=30)
    prev_time  = time.time()
    font       = cv2.FONT_HERSHEY_SIMPLEX

    master = Tk()
    master.title("Steering (experimental)")
    master.configure(bg="#111")
    master.resizable(False, False)
    wheel_canvas = Canvas(master, width=300, height=330, bg="#111",
                          highlightthickness=0)
    wheel_canvas.pack(padx=20, pady=20)

    cv2.namedWindow("SDC View [EXP]", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("SDC View [EXP]", 1280, 720)
    print("Camera mode — press 'q' quit | 'e' e-stop | 'o' toggle YOLO")

    while True:
        frame = capture.read()
        if frame is None:
            master.update()
            continue
        if capture.stopped:
            print("Camera stopped.")
            break

        lane_det.submit(frame)
        yolo_det.submit(frame)

        (left_pts, right_pts, steering_angle,
         lane_state, stop_line, isect_sigs) = lane_det.read()
        boxes_frame, detections = yolo_det.read()

        # Intersection persistence filter
        intersection_confirmed = isect_filter.update(isect_sigs)
        if intersection_confirmed:
            print(f"[INTERSECTION] Confirmed — signals: {isect_sigs.score}/4")

        # Gate stop line: only pass it through while actually approaching
        approaching      = (traffic.state == DriveState.APPROACHING_LINE)
        gated_stop_line  = stop_line and approaching

        brake_active, state_label, status_text = traffic.update(
            detections, kart, stop_line_detected=gated_stop_line)

        if kart and not brake_active:
            kart.steer(angle_to_fraction(steering_angle))

        display, direction, prev_time = build_display(
            frame, boxes_frame, left_pts, right_pts,
            lane_state, steering_angle,
            fps_window, prev_time, font,
            traffic.state, status_text,
            stop_line_active  = stop_line,
            intersection_sigs = isect_sigs)

        draw_steering_wheel(wheel_canvas, steering_angle)
        master.update()

        frame_count += 1
        if frame_count % PRINT_EVERY == 0:
            print(f"\n--- Frame {frame_count} | {direction:6s} {abs(steering_angle):.1f}° | "
                  f"L={'yes' if left_pts else 'no ':3s} R={'yes' if right_pts else 'no'} | "
                  f"StopLine={stop_line} | Isect={isect_sigs.score}/4 | "
                  f"YOLO={'ON ' if YOLO_ENABLED else 'OFF'} | Traffic: {status_text} ---")

        cv2.imshow("SDC View [EXP]", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('e'):
            if kart:
                kart.estop()
            print("ESTOP!")
        elif key == ord('o'):
            YOLO_ENABLED = not YOLO_ENABLED
            print(f"[YOLO] {'ON' if YOLO_ENABLED else 'OFF'}")

    lane_det.release()
    yolo_det.release()
    if kart:
        kart.close()
    capture.release()
    cv2.destroyAllWindows()
    master.destroy()


# ══════════════════════════════════════════════════════════════════════════════
# VIDEO MODE
# ══════════════════════════════════════════════════════════════════════════════

def run_on_video(path):
    global frame_count, YOLO_ENABLED
    frame_count = 0

    kart = make_kart()
    if kart:
        kart.release()

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"ERROR: Could not open video: {path}")
        return

    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30
    delay     = max(1, int(1000 / video_fps))
    total     = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {path}  |  {video_fps:.1f} fps  |  {total} frames")

    lane_det     = LaneDetector()
    yolo_det     = YoloDetector()
    traffic      = TrafficStateController()
    isect_filter = IntersectionDetector(confirm_frames=4, cooldown_frames=60)

    fps_window = deque(maxlen=30)
    prev_time  = time.time()
    font       = cv2.FONT_HERSHEY_SIMPLEX

    master = Tk()
    master.title("Steering (experimental)")
    master.configure(bg="#111")
    master.resizable(False, False)
    wheel_canvas = Canvas(master, width=300, height=330, bg="#111",
                          highlightthickness=0)
    wheel_canvas.pack(padx=20, pady=20)

    cv2.namedWindow("SDC View [EXP]", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("SDC View [EXP]", 1280, 720)
    print("Video mode — press 'q' quit | 'p' pause | 'e' e-stop | 'o' toggle YOLO")

    paused = False

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("End of video.")
                break

            lane_det.submit(frame)
            yolo_det.submit(frame)

            (left_pts, right_pts, steering_angle,
             lane_state, stop_line, isect_sigs) = lane_det.read()
            boxes_frame, detections = yolo_det.read()

            intersection_confirmed = isect_filter.update(isect_sigs)
            if intersection_confirmed:
                print(f"[INTERSECTION] Confirmed — signals: {isect_sigs.score}/4")

            approaching     = (traffic.state == DriveState.APPROACHING_LINE)
            gated_stop_line = stop_line and approaching

            brake_active, state_label, status_text = traffic.update(
                detections, kart, stop_line_detected=gated_stop_line)

            if kart and not brake_active:
                kart.steer(angle_to_fraction(steering_angle))

            display, direction, prev_time = build_display(
                frame, boxes_frame, left_pts, right_pts,
                lane_state, steering_angle,
                fps_window, prev_time, font,
                traffic.state, status_text,
                stop_line_active  = stop_line,
                intersection_sigs = isect_sigs)

            bar_w = int(display.shape[1] * frame_count / max(total, 1))
            cv2.rectangle(display,
                          (0, display.shape[0] - 6),
                          (bar_w, display.shape[0]),
                          (0, 200, 255), -1)

            draw_steering_wheel(wheel_canvas, steering_angle)
            master.update()

            frame_count += 1
            if frame_count % PRINT_EVERY == 0:
                print(f"\n--- Frame {frame_count}/{total} | {direction:6s} "
                      f"{abs(steering_angle):.1f}° | "
                      f"L={'yes' if left_pts else 'no ':3s} R={'yes' if right_pts else 'no'} | "
                      f"StopLine={stop_line} | Isect={isect_sigs.score}/4 | "
                      f"YOLO={'ON ' if YOLO_ENABLED else 'OFF'} | Traffic: {status_text} ---")

            cv2.imshow("SDC View [EXP]", display)

        key = cv2.waitKey(delay) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            paused = not paused
            print("Paused." if paused else "Resumed.")
        elif key == ord('e'):
            if kart:
                kart.estop()
            print("ESTOP!")
        elif key == ord('o'):
            YOLO_ENABLED = not YOLO_ENABLED
            print(f"[YOLO] {'ON' if YOLO_ENABLED else 'OFF'}")

        if paused:
            master.update()

    lane_det.release()
    yolo_det.release()
    if kart:
        kart.close()
    cap.release()
    cv2.destroyAllWindows()
    master.destroy()


# ══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python main_experimental.py camera")
        print("  python main_experimental.py camera 1")
        print("  python main_experimental.py video test.mp4")
        input("Press Enter to exit...")
        sys.exit(1)

    mode = sys.argv[1].lower()

    if mode == "camera":
        src = int(sys.argv[2]) if len(sys.argv) >= 3 else 1
        run_on_camera(source=src)
    elif mode == "video":
        if len(sys.argv) < 3:
            print("Please provide a video file path.")
            input("Press Enter to exit...")
            sys.exit(1)
        run_on_video(sys.argv[2])
    else:
        print("Invalid mode. Use 'camera' or 'video'.")
        input("Press Enter to exit...")