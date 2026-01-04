import cv2
import math
import time
import csv
import threading
import signal
import sys
from ultralytics import YOLO
from serial import Serial
from pyubx2 import UBXReader

MODEL_PATH = "/home/aryan/Desktop/best_ncnn_model"
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

# Global resources for cleanup
cap = None
out = None
gps = None
csv_f = None
shutdown_flag = False

class UBXGPSReader:
    def __init__(self, port="/dev/ttyAMA0", baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.reader = None
        self.running = False
        self.thread = None
        self.position = None
        self.lock = threading.Lock()
        self.has_fix = False
        
    def start(self):
        print(f"[GPS] Initializing GPS on {self.port} at {self.baudrate} baud...")
        self.serial = Serial(self.port, self.baudrate, timeout=1)
        self.reader = UBXReader(self.serial)
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("[GPS] GPS reader thread started")
        
    def _read_loop(self):
        while self.running:
            try:
                for _, msg in self.reader:
                    if not self.running:
                        break
                    if not msg or msg.identity != "NAV-PVT":
                        continue
                    
                    if msg.fixType >= 3 and msg.gnssFixOk and msg.numSV >= 5:
                        if not self.has_fix:
                            print(f"[GPS] â FIX ACQUIRED - {msg.numSV} satellites")
                            self.has_fix = True
                        
                        with self.lock:
                            self.position = {
                                "latitude": msg.lat,
                                "longitude": msg.lon,
                                "altitude": msg.hMSL / 1000.0,
                                "numSV": msg.numSV
                            }
                    else:
                        if self.has_fix:
                            print("[GPS] â FIX LOST")
                            self.has_fix = False
            except Exception as e:
                if self.running:  # Only print if not shutting down
                    print(f"[GPS] Error: {e}")
                time.sleep(0.1)
                
    def get_position(self):
        with self.lock:
            return self.position
    
    def stop(self):
        print("[GPS] Stopping GPS reader...")
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()
        print("[GPS] GPS reader stopped")

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

def cleanup_resources(frame_count, detection_count, start_time):
    """Clean up all resources and print summary"""
    global cap, out, gps, csv_f
    
    elapsed_total = int(time.time() - start_time) if start_time else 0
    
    print("\n" + "=" * 60)
    print("SHUTTING DOWN - SAVING DATA")
    print("=" * 60)
    
    if cap:
        cap.release()
        print("[CAMERA] â Camera released")
    
    if out:
        out.release()
        print("[VIDEO] â Video file saved")
    
    if gps:
        gps.stop()
    
    if csv_f:
        csv_f.close()
        print("[CSV] â CSV file closed")
    
    print("\n" + "=" * 60)
    print("RUN SUMMARY")
    print("=" * 60)
    print(f"Total runtime: {elapsed_total}s ({elapsed_total // 60}m {elapsed_total % 60}s)")
    print(f"Frames processed: {frame_count}")
    print(f"Humans detected: {detection_count}")
    print(f"Video saved to: {VIDEO_PATH}")
    print(f"CSV saved to: {CSV_PATH}")
    print("=" * 60)
    print("[SYSTEM] Shutdown complete. Safe to power off.\n")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global shutdown_flag
    print("\n\n[SYSTEM] Ctrl+C detected - stopping...")
    shutdown_flag = True

def main():
    global shutdown_flag, cap, out, gps, csv_f
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 60)
    print("SCOUT ACCURACY TEST - INITIALIZATION")
    print("=" * 60)
    
    frame_count = 0
    detection_count = 0
    start_time = None
    
    try:
        # Load model
        print("\n[MODEL] Loading YOLO model...")
        model = YOLO(MODEL_PATH, task="detect")
        print("[MODEL] â Model loaded successfully")
        
        # Initialize camera
        print("\n[CAMERA] Initializing camera...")
        print(f"[CAMERA] Device: {CAMERA_DEV}")
        print(f"[CAMERA] Resolution: {FRAME_W}x{FRAME_H}")
        cap = cv2.VideoCapture(CAMERA_DEV, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        
        if not cap.isOpened():
            raise RuntimeError("Camera failed to open")
        
        print("[CAMERA] â Camera initialized successfully")
        
        # Initialize video writer
        print(f"\n[VIDEO] Initializing video recording: {VIDEO_PATH}")
        out = cv2.VideoWriter(
            VIDEO_PATH,
            cv2.VideoWriter_fourcc(*"XVID"),
            10.0,
            (FRAME_W, FRAME_H),
        )
        print("[VIDEO] â Video writer ready")
        
        # Initialize GPS
        print("\n[GPS] Starting GPS system...")
        gps = UBXGPSReader("/dev/ttyAMA0", 230400)
        gps.start()
        
        # Wait for initial GPS fix
        print("[GPS] Waiting for GPS fix...")
        fix_wait_start = time.time()
        while not gps.get_position() and not shutdown_flag:
            elapsed = int(time.time() - fix_wait_start)
            print(f"[GPS] Waiting for fix... ({elapsed}s)", end='\r')
            time.sleep(0.5)
        
        if shutdown_flag:
            print("\n[SYSTEM] Shutdown requested during GPS wait")
            return
        
        pos = gps.get_position()
        print(f"\n[GPS] â GPS fix obtained")
        print(f"[GPS] Position: {pos['latitude']:.7f}, {pos['longitude']:.7f}")
        print(f"[GPS] Satellites: {pos['numSV']}")
        
        # Initialize CSV
        print(f"\n[CSV] Creating output file: {CSV_PATH}")
        csv_f = open(CSV_PATH, "w", newline="")
        writer = csv.writer(csv_f)
        writer.writerow(["lat", "lon", "alt"])
        print("[CSV] â CSV file ready")
        
        print("\n" + "=" * 60)
        print("ALL SYSTEMS READY - READY TO FLY!")
        print("=" * 60)
        print(f"Run time: {RUN_TIME // 60} minutes")
        print(f"Confidence threshold: {CONF_THRES}")
        print(f"Confirm time: {CONFIRM_TIME}s")
        print("Press Ctrl+C to stop and save\n")
        
        tracks = []
        next_id = 1
        start_time = time.time()
        last_ts = time.time()
        
        print("[SCOUT] Detection started...\n")
        
        while (time.time() - start_time < RUN_TIME) and not shutdown_flag:
            ret, frame = cap.read()
            if not ret:
                continue
            
            frame_count += 1
            out.write(frame)
            
            pos = gps.get_position()
            if not pos:
                continue
            
            lat0 = pos["latitude"]
            lon0 = pos["longitude"]
            
            now = time.time()
            dt = now - last_ts
            last_ts = now
            
            # Check for shutdown before expensive inference
            if shutdown_flag:
                break
            
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
                    detection_count += 1
                    elapsed = int(time.time() - start_time)
                    print(f"ð¨ [HUMAN #{detection_count}] CONFIRMED ð¨")
                    print(f"   GPS: {lat:.7f}, {lon:.7f}")
                    print(f"   Time: {elapsed}s | Frame: {frame_count}\n")
        
        if not shutdown_flag:
            print("\n[SYSTEM] Run time completed normally")
        
    except KeyboardInterrupt:
        # This shouldn't trigger due to signal handler, but just in case
        print("\n[SYSTEM] Keyboard interrupt received")
        shutdown_flag = True
        
    except Exception as e:
        print(f"\n[ERROR] Fatal error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        cleanup_resources(frame_count, detection_count, start_time)

if __name__ == "__main__":
    main()
