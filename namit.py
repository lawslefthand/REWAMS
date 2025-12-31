import cv2
import math
import time
import csv
from ultralytics import YOLO
from gps_reader import GPSReader

MODEL_PATH = "/home/aryan/Downloads/best_ncnn_model"
CAMERA_DEV = "/dev/video0"

FRAME_W = 640
FRAME_H = 480

ALTITUDE_M = 20.0
FOV_H_DEG = 90.0

CONF_THRES = 0.45
CONFIRM_TIME = 2.0
LOST_TIME = 4.0
MATCH_RADIUS = 120

RUN_TIME = 15 * 60

VIDEO_PATH = "scout_run.avi"
CSV_PATH = "humans.csv"


def bbox_to_ground(cx, cy):
    dx = cx - FRAME_W / 2
    dy = cy - FRAME_H / 2
    fov_h = math.radians(FOV_H_DEG)
    ground_w = 2 * ALTITUDE_M * math.tan(fov_h / 2)
    fov_v = 2 * math.atan((FRAME_H / FRAME_W) * math.tan(fov_h / 2))
    ground_h = 2 * ALTITUDE_M * math.tan(fov_v / 2)
    mx = ground_w / FRAME_W
    my = ground_h / FRAME_H
    return -dy * my, dx * mx


def project(lat0, lon0, north, east):
    dlat = north / 111320.0
    dlon = east / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def main():
    print("Loading model...")
    model = YOLO(MODEL_PATH, task="detect")
    print("Model loaded")

    cap = cv2.VideoCapture(CAMERA_DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        raise RuntimeError("Camera open failed")

    out = cv2.VideoWriter(
        VIDEO_PATH,
        cv2.VideoWriter_fourcc(*"XVID"),
        10.0,
        (FRAME_W, FRAME_H),
    )

    gps = GPSReader("/dev/ttyAMA0", 9600)
    gps.start()

    csv_f = open(CSV_PATH, "w", newline="")
    writer = csv.writer(csv_f)
    writer.writerow(["lat", "lon", "alt"])

    tracks = []
    next_id = 1
    start = time.time()
    last_ts = time.time()

    print("Scout running headless")

    while time.time() - start < RUN_TIME:
        ret, frame = cap.read()
        if not ret:
            continue

        out.write(frame)

        pos = gps.get_position()
        if not pos:
            continue

        lat0 = pos["latitude"]
        lon0 = pos["longitude"]

        now = time.time()
        dt = now - last_ts
        last_ts = now

        results = model(frame, conf=CONF_THRES, verbose=False)

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
                if d < MATCH_RADIUS and d < best_d:
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
            if now - t["last"] > LOST_TIME:
                tracks.remove(t)
                continue

            if not t["done"] and t["time"] >= CONFIRM_TIME:
                north, east = bbox_to_ground(t["px"][0], t["px"][1])
                lat, lon = project(lat0, lon0, north, east)
                writer.writerow([f"{lat:.7f}", f"{lon:.7f}", f"{ALTITUDE_M:.1f}"])
                csv_f.flush()
                t["done"] = True
                print(f"CONFIRMED | {lat:.7f}, {lon:.7f}")

    cap.release()
    out.release()
    gps.stop()
    csv_f.close()
    print("Finished")


if __name__ == "__main__":
    main()
