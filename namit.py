import cv2
import math
import time
import csv
from ultralytics import YOLO
from gps_reader import GPSReader

# ===============================
# CONFIG
# ===============================
MODEL_PATH = "/home/aryan/Downloads/best_ncnn_model"

CAPTURE_WIDTH = 416
CAPTURE_HEIGHT = 416
TARGET_FPS = 10

ALTITUDE_M = 20.0
FOV_HORIZONTAL_DEG = 90.0

CONF_THRESHOLD = 0.45
CONFIRM_TIME = 2.0
TRACK_LOST_TIME = 4.0
MATCH_RADIUS_PX = 120

RUN_TIME = 15 * 60

VIDEO_OUT = "scout_run.mp4"
CSV_OUT = "humans.csv"

FRAME_INTERVAL = 1.0 / TARGET_FPS


# ===============================
# CAMERA (Pi Camera via GStreamer)
# ===============================
def open_pi_camera():
    gst = (
        "libcamerasrc ! "
        f"video/x-raw,width={CAPTURE_WIDTH},height={CAPTURE_HEIGHT},framerate={TARGET_FPS}/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true"
    )

    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Pi Camera failed via GStreamer")

    return cap


# ===============================
# GEOMETRY
# ===============================
def bbox_to_body_m(cx, cy):
    dx = cx - CAPTURE_WIDTH / 2
    dy = cy - CAPTURE_HEIGHT / 2

    fov_h = math.radians(FOV_HORIZONTAL_DEG)
    ground_w = 2 * ALTITUDE_M * math.tan(fov_h / 2)
    fov_v = 2 * math.atan((CAPTURE_HEIGHT / CAPTURE_WIDTH) * math.tan(fov_h / 2))
    ground_h = 2 * ALTITUDE_M * math.tan(fov_v / 2)

    mx = ground_w / CAPTURE_WIDTH
    my = ground_h / CAPTURE_HEIGHT

    dx_m = dx * mx
    dy_m = dy * my

    # forward, right
    return -dy_m, dx_m


def project(lat0, lon0, x_m, y_m):
    dlat = x_m / 111320.0
    dlon = y_m / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


# ===============================
# MAIN
# ===============================
def main():
    print("Loading model...")
    model = YOLO(MODEL_PATH, task="detect")
    print("Model loaded")

    cap = open_pi_camera()

    out = cv2.VideoWriter(
        VIDEO_OUT,
        cv2.VideoWriter_fourcc(*"mp4v"),
        TARGET_FPS,
        (CAPTURE_WIDTH, CAPTURE_HEIGHT),
    )

    if not out.isOpened():
        print("Video writer failed, disabling recording")
        out = None

    gps = GPSReader("/dev/ttyAMA0", 9600)
    gps.start()

    csv_file = open(CSV_OUT, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["lat", "lon", "alt"])

    tracks = []
    next_id = 1

    start_time = time.time()
    last_frame_write = 0.0
    last_loop_time = time.time()

    print("Scout running")

    while time.time() - start_time < RUN_TIME:
        ret, frame = cap.read()
        if not ret:
            continue

        now = time.time()
        dt = now - last_loop_time
        last_loop_time = now

        if out and now - last_frame_write >= FRAME_INTERVAL:
            out.write(frame)
            last_frame_write = now

        pos = gps.get_position()
        if not pos:
            continue

        lat0 = pos["latitude"]
        lon0 = pos["longitude"]

        results = model(frame, conf=CONF_THRESHOLD, verbose=False)

        detections = []
        if results and results[0].boxes:
            for b in results[0].boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                detections.append(((x1 + x2) / 2, (y1 + y2) / 2))

        used = set()

        for cx, cy in detections:
            best = None
            best_d = 1e9

            for i, t in enumerate(tracks):
                if i in used:
                    continue
                d = math.hypot(cx - t["px"][0], cy - t["px"][1])
                if d < MATCH_RADIUS_PX and d < best_d:
                    best = i
                    best_d = d

            if best is not None:
                t = tracks[best]
                t["px"] = (cx, cy)
                t["last"] = now
                t["time"] += dt
                used.add(best)
            else:
                tracks.append({
                    "id": next_id,
                    "px": (cx, cy),
                    "last": now,
                    "time": dt,
                    "done": False
                })
                next_id += 1

        for t in list(tracks):
            if now - t["last"] > TRACK_LOST_TIME:
                tracks.remove(t)
                continue

            if not t["done"] and t["time"] >= CONFIRM_TIME:
                xb, yb = bbox_to_body_m(t["px"][0], t["px"][1])
                lat, lon = project(lat0, lon0, xb, yb)

                writer.writerow([f"{lat:.7f}", f"{lon:.7f}", f"{ALTITUDE_M:.1f}"])
                csv_file.flush()

                t["done"] = True
                print(f"CONFIRMED | {lat:.7f}, {lon:.7f}")

    cap.release()
    if out:
        out.release()
    gps.stop()
    csv_file.close()

    print("Scout finished")


if __name__ == "__main__":
    main()
