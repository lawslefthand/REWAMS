import time
import math
import csv
import cv2
from ultralytics import YOLO
from gps_reader import GPSReader
from picamera2 import Picamera2

MODEL_PATH = "/home/aryan/Downloads/best_ncnn_model"

WIDTH = 416
HEIGHT = 416

ALTITUDE_M = 20.0
FOV_H_DEG = 90.0

CONF_TH = 0.45
CONFIRM_TIME = 2.0
TRACK_LOST = 4.0
MATCH_RADIUS = 120

RUN_TIME = 15 * 60
FPS_SAVE = 10

VIDEO_OUT = "scout_run.mp4"
CSV_OUT = "humans.csv"


def bbox_to_body(cx, cy):
    dx = cx - WIDTH / 2
    dy = cy - HEIGHT / 2
    fov_h = math.radians(FOV_H_DEG)
    gw = 2 * ALTITUDE_M * math.tan(fov_h / 2)
    gv = 2 * ALTITUDE_M * math.tan(math.atan((HEIGHT / WIDTH) * math.tan(fov_h / 2)))
    mx = gw / WIDTH
    my = gv / HEIGHT
    return -dy * my, dx * mx


def project(lat0, lon0, north, east):
    dlat = north / 111320.0
    dlon = east / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def main():
    print("Loading model...")
    model = YOLO(MODEL_PATH, task="detect")
    print("Model loaded")

    cam = Picamera2()
    cam.configure(
        cam.create_video_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
        )
    )
    cam.start()
    time.sleep(2)

    out = cv2.VideoWriter(
        VIDEO_OUT,
        cv2.VideoWriter_fourcc(*"mp4v"),
        FPS_SAVE,
        (WIDTH, HEIGHT),
    )

    gps = GPSReader("/dev/ttyAMA0", 9600)
    gps.start()

    csvf = open(CSV_OUT, "w", newline="")
    writer = csv.writer(csvf)
    writer.writerow(["lat", "lon", "alt"])

    tracks = []
    next_id = 1

    start = time.time()
    last_write = 0
    last_loop = time.time()

    print("Scout running headless...")

    try:
        while time.time() - start < RUN_TIME:
            frame = cam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            now = time.time()
            dt = now - last_loop
            last_loop = now

            if now - last_write >= 1 / FPS_SAVE:
                out.write(frame)
                last_write = now

            pos = gps.get_position()
            if not pos:
                continue

            lat0 = pos["latitude"]
            lon0 = pos["longitude"]

            res = model(frame, conf=CONF_TH, verbose=False)

            dets = []
            if res and res[0].boxes:
                for b in res[0].boxes:
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    dets.append(((x1 + x2) / 2, (y1 + y2) / 2))

            used = set()

            for cx, cy in dets:
                best, best_d = None, 1e9
                for i, t in enumerate(tracks):
                    if i in used:
                        continue
                    d = math.hypot(cx - t["px"][0], cy - t["px"][1])
                    if d < MATCH_RADIUS and d < best_d:
                        best, best_d = i, d

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
                if now - t["last"] > TRACK_LOST:
                    tracks.remove(t)
                    continue

                if not t["done"] and t["time"] >= CONFIRM_TIME:
                    north, east = bbox_to_body(t["px"][0], t["px"][1])
                    lat, lon = project(lat0, lon0, north, east)
                    writer.writerow([f"{lat:.7f}", f"{lon:.7f}", f"{ALTITUDE_M:.1f}"])
                    csvf.flush()
                    t["done"] = True
                    print(f"CONFIRMED | {lat:.7f}, {lon:.7f}")

    finally:
        cam.stop()
        out.release()
        gps.stop()
        csvf.close()
        print("Scout finished cleanly")


if __name__ == "__main__":
    main()
