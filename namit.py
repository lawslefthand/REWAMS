import cv2
import math
import time
import csv
import signal
import sys
import os
import threading
from ultralytics import YOLO
from serial import Serial
from pyubx2 import UBXReader

MODEL_PATH = "/home/aryan/Desktop/best_ncnn_model"
CAMERA_DEV = "/dev/video0"
GPS_PORT = "/dev/ttyAMA0"
GPS_BAUD = 230400

OUT_DIR = "/home/aryan/Desktop/output"
os.makedirs(OUT_DIR, exist_ok=True)

VIDEO_PATH = f"{OUT_DIR}/scout_output.avi"
CSV_PATH = f"{OUT_DIR}/detections.csv"

FRAME_W = 640
FRAME_H = 480
ALTITUDE_M = 20.0
FOV_H_DEG = 90.0

CONF_THRES = 0.45
CONFIRM_TIME = 2.0
LOST_TIME = 4.0
MATCH_RADIUS = 120

shutdown_flag = False


class UBXGPSReader:
    def __init__(self):
        self.serial = None
        self.reader = None
        self.running = False
        self.thread = None
        self.position = None
        self.lock = threading.Lock()

    def start(self):
        self.serial = Serial(GPS_PORT, GPS_BAUD, timeout=1)
        self.reader = UBXReader(self.serial)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                for _, msg in self.reader:
                    if not self.running:
                        break
                    if msg and msg.identity == "NAV-PVT":
                        if msg.fixType >= 3 and msg.gnssFixOk and msg.numSV >= 5:
                            with self.lock:
                                self.position = {
                                    "lat": msg.lat,
                                    "lon": msg.lon
                                }
            except:
                time.sleep(0.1)

    def get(self):
        with self.lock:
            return self.position

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()


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


def signal_handler(sig, frame):
    global shutdown_flag
    shutdown_flag = True
    print("\n[SYSTEM] Ctrl+C received — finishing safely...")


signal.signal(signal.SIGINT, signal_handler)


def main():
    gps = None
    cap = None
    out = None
    csv_f = None

    try:
        print("[SYSTEM] Starting scout system")

        gps = UBXGPSReader()
        gps.start()

        print("[GPS] Waiting for fix...")
        while gps.get() is None:
            time.sleep(0.5)

        print("[GPS] Fix acquired")

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
            (FRAME_W, FRAME_H)
        )

        csv_f = open(CSV_PATH, "w", newline="")
        writer = csv.writer(csv_f)
        writer.writerow(["id", "lat", "lon", "alt"])

        model = YOLO(MODEL_PATH, task="detect")

        tracks = []
        next_id = 1
        last_ts = time.time()

        print("[SYSTEM] Running — Ctrl+C to stop")

        while not shutdown_flag:
            ret, frame = cap.read()
            if not ret:
                continue

            pos = gps.get()
            out.write(frame)

            if not pos:
                continue

            lat0 = pos["lat"]
            lon0 = pos["lon"]

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
                    tracks[best]["px"] = (cx, cy)
                    tracks[best]["last"] = now
                    tracks[best]["time"] += dt
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
                    writer.writerow([t["id"], f"{lat:.7f}", f"{lon:.7f}", ALTITUDE_M])
                    csv_f.flush()
                    t["done"] = True
                    print(f"[CONFIRMED] ID {t['id']} → {lat:.7f}, {lon:.7f}")

    finally:
        print("[SYSTEM] Saving and shutting down")

        if cap:
            cap.release()
        if out:
            out.release()
        if gps:
            gps.stop()
        if csv_f:
            csv_f.close()

        print(f"[VIDEO] Saved → {VIDEO_PATH}")
        print(f"[CSV] Saved → {CSV_PATH}")
        print("[SYSTEM] Safe to power off")


if __name__ == "__main__":
    main()
