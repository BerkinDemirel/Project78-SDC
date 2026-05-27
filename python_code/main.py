from ultralytics import YOLO
import cv2
import numpy as np
import time
import threading
import math
import sys
from collections import deque
from tkinter import *
from kart_control import KartController, angle_to_fraction


# todo!! make line more curvable and make noise filter more robus
# ── Kart control ──────────────────────────────────────────────────────────────

KART_PORT    = "/dev/ttyUSB0"   # Linux (Jetson). Change to "COM3" etc. for Windows.
KART_ENABLED = True             # set False to run vision-only without hardware

# ── YOLO model ────────────────────────────────────────────────────────────────

YOLO_ENABLED = False             # set False to skip object detection entirely
model        = YOLO('best.pt') if YOLO_ENABLED else None

# ── Steering settings ─────────────────────────────────────────────────────────

STEERING_ANGLE_LIMIT = 90.0
ROI_TOP              = 0.8
ROI_BOTTOM           = 0.6
ROI_LEFT             = 0
ROI_RIGHT            = 1

# ── Shared state ──────────────────────────────────────────────────────────────

prev_left_line  = None
prev_right_line = None
error_history   = []
MAX_HISTORY_LEN = 5

# ── Print throttle ────────────────────────────────────────────────────────────

PRINT_EVERY = 10
frame_count  = 0


# ──────────────────────────────────────────────
# FRAME CAPTURE THREAD
# ──────────────────────────────────────────────

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
            raise RuntimeError(f"Error: Could not open camera source: {self.source}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS,          30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        return cap

    def _reader(self):
        retries = 0
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                retries += 1
                print(f"Failed to grab frame (attempt {retries}/{self.max_retries})")
                if retries >= self.max_retries:
                    print("Max retries reached — attempting camera reconnect...")
                    self.cap.release()
                    time.sleep(1.0)
                    try:
                        self.cap = self._open_cap()
                        retries  = 0
                        print("Camera reconnected.")
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


# ──────────────────────────────────────────────
# LANE DETECTION THREAD
# ──────────────────────────────────────────────

class LaneDetector:
    """Runs lane detection on its own thread. Always processes the latest frame."""

    def __init__(self):
        self.input_frame    = None
        self.left_line      = None
        self.right_line     = None
        self.steering_angle = 0.0
        self.input_lock     = threading.Lock()
        self.output_lock    = threading.Lock()
        self.stopped        = False
        self.thread         = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, frame):
        with self.input_lock:
            self.input_frame = frame.copy()

    def _run(self):
        while not self.stopped:
            with self.input_lock:
                frame = self.input_frame
                self.input_frame = None

            if frame is None:
                time.sleep(0.001)
                continue

            left, right = _detect_lines_worker(frame)
            angle       = _compute_steering(frame.shape[1], left, right)

            with self.output_lock:
                self.left_line      = left
                self.right_line     = right
                self.steering_angle = angle

    def read(self):
        with self.output_lock:
            return self.left_line, self.right_line, self.steering_angle

    def release(self):
        self.stopped = True
        self.thread.join(timeout=3)


# ──────────────────────────────────────────────
# YOLO DETECTION THREAD
# ──────────────────────────────────────────────

class YoloDetector:
    """Runs YOLO inference on its own thread on CUDA. Always processes the latest frame."""

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
                frame = self.input_frame
                self.input_frame = None

            if frame is None:
                time.sleep(0.001)
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


# ──────────────────────────────────────────────
# LANE DETECTION LOGIC
# ──────────────────────────────────────────────

def smooth_line(current, previous, alpha=0.1):
    if current is None:
        return previous
    if previous is None:
        return current
    return tuple(int(alpha * c + (1 - alpha) * p) for c, p in zip(current, previous))


def make_line_points(height, line_params):
    if not line_params:
        return None
    slope, intercept = np.mean(line_params, axis=0)
    y1 = int(height * ROI_BOTTOM)
    y2 = int(height * ROI_TOP)
    if abs(slope) < 1e-6:
        return None
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return (x1, y1, x2, y2)


def _detect_lines_worker(frame):
    global prev_left_line, prev_right_line

    hls        = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    white_mask = cv2.inRange(hls, np.array([0, 115, 0]), np.array([255, 255, 255]))

    lab             = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l_channel, a, b = cv2.split(lab)
    clahe           = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    cl              = clahe.apply(l_channel)

    blur        = cv2.GaussianBlur(cl, (7, 7), 1)
    edges       = cv2.Canny(blur, 70, 180)
    white_edges = cv2.bitwise_and(edges, white_mask)

    h, w    = frame.shape[:2]
    mask    = np.zeros_like(white_edges)
    polygon = np.array([[
        (int(w * ROI_LEFT),  int(h * ROI_BOTTOM)),
        (int(w * ROI_LEFT),  int(h * ROI_TOP)),
        (int(w * ROI_RIGHT), int(h * ROI_TOP)),
        (int(w * ROI_RIGHT), int(h * ROI_BOTTOM))
    ]], dtype=np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped = cv2.bitwise_and(white_edges, mask)

    lines = cv2.HoughLinesP(cropped, rho=1, theta=np.pi / 180,
                            threshold=40, minLineLength=40, maxLineGap=120)

    left_fits, right_fits = [], []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope     = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            if abs(slope) < 0.15:
                continue
            if slope < 0:
                left_fits.append((slope, intercept))
            else:
                right_fits.append((slope, intercept))

    left_line  = smooth_line(make_line_points(h, left_fits),  prev_left_line)
    right_line = smooth_line(make_line_points(h, right_fits), prev_right_line)

    prev_left_line  = left_line
    prev_right_line = right_line

    return left_line, right_line


def _valid_left(x, w):  return 0 < x < w * 0.55
def _valid_right(x, w): return w * 0.45 < x < w


def _compute_steering(frame_width, left_line, right_line):
    center_x = frame_width // 2
    left_ok  = left_line  is not None and _valid_left(left_line[0],   frame_width)
    right_ok = right_line is not None and _valid_right(right_line[0], frame_width)

    if left_ok and right_ok:
        lane_center = (left_line[0] + right_line[0]) // 2
    elif left_ok:
        lane_center = left_line[0]  + int(0.22 * frame_width)
    elif right_ok:
        lane_center = right_line[0] - int(0.22 * frame_width)
    else:
        error_history.append(error_history[-1] if error_history else 0.0)
        if len(error_history) > MAX_HISTORY_LEN:
            error_history.pop(0)
        smoothed = sum(error_history) / len(error_history)
        return max(-STEERING_ANGLE_LIMIT, min(STEERING_ANGLE_LIMIT,
                                              smoothed * STEERING_ANGLE_LIMIT))

    error      = lane_center - center_x
    max_offset = frame_width * 0.25
    normalized = max(-1.0, min(1.0, error / max_offset))
    if abs(normalized) < 0.03:
        normalized = 0.0

    error_history.append(normalized)
    if len(error_history) > MAX_HISTORY_LEN:
        error_history.pop(0)

    smoothed = sum(error_history) / len(error_history)
    return max(-STEERING_ANGLE_LIMIT, min(STEERING_ANGLE_LIMIT,
                                          smoothed * STEERING_ANGLE_LIMIT))


# ──────────────────────────────────────────────
# STEERING WHEEL WIDGET
# ──────────────────────────────────────────────

def draw_steering_wheel(canvas, angle, width=300, height=300):
    canvas.delete("all")
    cx, cy = width // 2, height // 2
    radius = min(cx, cy) - 20

    canvas.create_oval(cx - radius, cy - radius,
                       cx + radius, cy + radius,
                       outline="#444", width=18, fill="#222")
    hub = radius * 0.18
    canvas.create_oval(cx - hub, cy - hub,
                       cx + hub, cy + hub,
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


# ──────────────────────────────────────────────
# COMPOSITE FRAME BUILDER
# ──────────────────────────────────────────────

def build_display(base_frame, boxes_frame, left_line, right_line,
                  steering_angle, fps_window, prev_time, font):
    display = boxes_frame.copy() if boxes_frame is not None else base_frame.copy()

    line_image = np.zeros_like(display)
    if left_line:
        cv2.line(line_image, left_line[:2],  left_line[2:],  (0, 255, 0), 10)
    if right_line:
        cv2.line(line_image, right_line[:2], right_line[2:], (0, 255, 0), 10)
    display = cv2.addWeighted(display, 1.0, line_image, 0.8, 0)

    direction = "RIGHT" if steering_angle > 0 else "LEFT" if steering_angle < 0 else "STRAIGHT"
    cv2.putText(display, f"Steer: {steering_angle:.1f}deg ({direction})",
                (10, 40), font, 1.0, (0, 255, 255), 2, cv2.LINE_AA)

    current_time = time.time()
    fps_window.append(1 / (current_time - prev_time))
    avg_fps = sum(fps_window) / len(fps_window)
    cv2.putText(display, f"FPS: {avg_fps:.1f}",
                (10, 80), font, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

    return display, direction, current_time


# ──────────────────────────────────────────────
# KART HELPER
# ──────────────────────────────────────────────

def make_kart():
    """Connect to kart hardware. Returns KartController or None."""
    if not KART_ENABLED:
        return None
    kart = KartController(port=KART_PORT)
    if not kart.connected:
        print("WARNING: Arduino not connected — running without hardware output.")
        return None
    return kart


# ──────────────────────────────────────────────
# CAMERA MODE
# ──────────────────────────────────────────────

def run_on_camera(source=1):
    global frame_count
    frame_count = 0
    error_history.clear()

    kart = make_kart()
    if kart:
        kart.release()

    capture  = FrameCapture(source=source, max_retries=5)
    lane_det = LaneDetector()
    yolo_det = YoloDetector()

    fps_window = deque(maxlen=30)
    prev_time  = time.time()
    font       = cv2.FONT_HERSHEY_SIMPLEX

    master = Tk()
    master.title("Steering")
    master.configure(bg="#111")
    master.resizable(False, False)
    wheel_canvas = Canvas(master, width=300, height=330, bg="#111", highlightthickness=0)
    wheel_canvas.pack(padx=20, pady=20)

    cv2.namedWindow("SDC View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("SDC View", 1280, 720)
    print("Camera mode — press 'q' to quit, 'e' for e-stop.")

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

        left_line, right_line, steering_angle = lane_det.read()
        boxes_frame, detections               = yolo_det.read()

        if kart:
            kart.steer(angle_to_fraction(steering_angle))

        display, direction, prev_time = build_display(
            frame, boxes_frame, left_line, right_line,
            steering_angle, fps_window, prev_time, font)

        draw_steering_wheel(wheel_canvas, steering_angle)
        master.update()

        frame_count += 1
        if frame_count % PRINT_EVERY == 0:
            print(f"\n--- Frame {frame_count} | Steer {direction:6s} {abs(steering_angle):.1f}° | "
                  f"left={'yes' if left_line else 'no':3s} right={'yes' if right_line else 'no'} ---")
            if detections:
                for d in detections:
                    x1, y1, x2, y2 = d['bbox']
                    print(f"  {d['label']:20s}  conf: {d['confidence']:.2f}  "
                          f"box: ({x1},{y1})->({x2},{y2})")
            else:
                print("  No objects detected")

        cv2.imshow("SDC View", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('e'):
            if kart:
                kart.estop()
                print("ESTOP triggered!")

    lane_det.release()
    yolo_det.release()
    if kart:
        kart.close()
    capture.release()
    cv2.destroyAllWindows()
    master.destroy()


# ──────────────────────────────────────────────
# VIDEO MODE
# ──────────────────────────────────────────────

def run_on_video(path):
    global frame_count
    frame_count = 0
    error_history.clear()

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

    lane_det = LaneDetector()
    yolo_det = YoloDetector()

    fps_window = deque(maxlen=30)
    prev_time  = time.time()
    font       = cv2.FONT_HERSHEY_SIMPLEX

    master = Tk()
    master.title("Steering")
    master.configure(bg="#111")
    master.resizable(False, False)
    wheel_canvas = Canvas(master, width=300, height=330, bg="#111", highlightthickness=0)
    wheel_canvas.pack(padx=20, pady=20)

    cv2.namedWindow("SDC View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("SDC View", 1280, 720)
    print("Video mode — press 'q' to quit, 'p' to pause/resume, 'e' for e-stop.")

    paused = False

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("End of video.")
                break

            lane_det.submit(frame)
            yolo_det.submit(frame)

            left_line, right_line, steering_angle = lane_det.read()
            boxes_frame, detections               = yolo_det.read()

            if kart:
                kart.steer(angle_to_fraction(steering_angle))

            display, direction, prev_time = build_display(
                frame, boxes_frame, left_line, right_line,
                steering_angle, fps_window, prev_time, font)

            progress = frame_count / max(total, 1)
            bar_w    = int(display.shape[1] * progress)
            cv2.rectangle(display, (0, display.shape[0] - 6),
                          (bar_w, display.shape[0]), (0, 200, 255), -1)

            draw_steering_wheel(wheel_canvas, steering_angle)
            master.update()

            frame_count += 1
            if frame_count % PRINT_EVERY == 0:
                print(f"\n--- Frame {frame_count}/{total} | Steer {direction:6s} "
                      f"{abs(steering_angle):.1f}° | "
                      f"left={'yes' if left_line else 'no':3s} "
                      f"right={'yes' if right_line else 'no'} ---")
                if detections:
                    for d in detections:
                        x1, y1, x2, y2 = d['bbox']
                        print(f"  {d['label']:20s}  conf: {d['confidence']:.2f}  "
                              f"box: ({x1},{y1})->({x2},{y2})")
                else:
                    print("  No objects detected")

            cv2.imshow("SDC View", display)

        key = cv2.waitKey(delay) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            paused = not paused
            print("Paused." if paused else "Resumed.")
        elif key == ord('e'):
            if kart:
                kart.estop()
                print("ESTOP triggered!")

        if paused:
            master.update()

    lane_det.release()
    yolo_det.release()
    if kart:
        kart.close()
    cap.release()
    cv2.destroyAllWindows()
    master.destroy()


# ──────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python sdc.py camera")
        print("  python sdc.py camera 1          (specify device index)")
        print("  python sdc.py video test.mp4")
        input("Press Enter to exit...")
        sys.exit(1)

    mode = sys.argv[1].lower()

    if mode == "camera":
        src = int(sys.argv[2]) if len(sys.argv) >= 3 else 1
        run_on_camera(source=src)

    elif mode == "video":
        if len(sys.argv) < 3:
            print("Please provide a video file path.")
            print("Example: python sdc.py video test.mp4")
            input("Press Enter to exit...")
            sys.exit(1)
        run_on_video(sys.argv[2])

    else:
        print("Invalid mode. Use 'camera' or 'video'.")
        input("Press Enter to exit...")