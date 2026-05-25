"""
main_lane_only.py — Visualize raw edge detection output only.

Keys:
  q  — quit
  p  — pause / resume  (video mode only)
"""

import cv2
import numpy as np
import sys

DOT_MIN_AREA = 5000


def _lane_mask(frame):
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    white  = cv2.inRange(hls, np.array([0,   130,   0]),
                              np.array([255, 255,  60]))
    yellow = cv2.inRange(hls, np.array([15,   80,  80]),
                              np.array([35,  255, 255]))

    dot_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    white_clean = cv2.morphologyEx(white, cv2.MORPH_OPEN, dot_kernel, iterations=1)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(white_clean, connectivity=8)
    clean = np.zeros_like(white_clean)
    for lbl in range(1, num_labels):
        if stats[lbl, cv2.CC_STAT_AREA] >= DOT_MIN_AREA:
            clean[labels == lbl] = 255
    white_clean = clean

    return cv2.bitwise_or(white_clean, yellow)


def run_on_camera(source=1):
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera: {source}"); return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS,          30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    cv2.namedWindow("Edge Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Edge Detection", 1280, 720)
    print("Camera mode — press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame"); continue

        mask = _lane_mask(frame)
        cv2.imshow("Edge Detection", mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def run_on_video(path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"ERROR: Could not open video: {path}"); return

    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30
    delay     = max(1, int(1000 / video_fps))
    print(f"Video: {path}  |  {video_fps:.1f} fps")

    cv2.namedWindow("Edge Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Edge Detection", 1280, 720)
    print("Video mode — press 'q' quit | 'p' pause")

    paused = False

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("End of video."); break

            mask = _lane_mask(frame)
            cv2.imshow("Edge Detection", mask)

        key = cv2.waitKey(delay) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            paused = not paused
            print("Paused." if paused else "Resumed.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python main_lane_only.py camera")
        print("  python main_lane_only.py camera 1")
        print("  python main_lane_only.py video test.mp4")
        sys.exit(1)

    mode = sys.argv[1].lower()

    if mode == "camera":
        src = int(sys.argv[2]) if len(sys.argv) >= 3 else 1
        run_on_camera(source=src)
    elif mode == "video":
        if len(sys.argv) < 3:
            print("Please provide a video file path.")
            sys.exit(1)
        run_on_video(sys.argv[2])
    else:
        print("Invalid mode. Use 'camera' or 'video'.")
        sys.exit(1)
