#!/usr/bin/env python3
"""
SCOUT DRONE v3 - Competition Mode with Heading Compensation
Handles drone rotation during flight
"""
import cv2
import math
import time
import csv
import signal
import numpy as np
from ultralytics import YOLO
from serial import Serial
from pyubx2 import UBXReader
import threading

# ===== CONFIGURATION =====
MODEL_PATH = "/home/aryan/Desktop/best_ncnn_model"
CAMERA_DEV = "/dev/video0"
GPS_PORT = "/dev/ttyAMA0"
GPS_BAUD = 230400
HOMOGRAPHY_PATH = "homography.npy"
FRAME_SIZE_PATH = "calibration_frame_size.npy"

FRAME_W = 640
FRAME_H = 480
ALTITUDE_M = 20
CONF_THRES = 0.45
CONFIRM_TIME = 2.0
LOST_TIME = 4.0
MATCH_RADIUS = 120

VIDEO_PATH = "competition_run.avi"
CSV_PATH = "targets.csv"

# ===== HEADING CONFIGURATION =====
# The heading (degrees) the camera was pointing during calibration
# 0 = North, 90 = East, 180 = South, 270 = West
CALIBRATION_HEADING = 0.0  # Camera was pointing NORTH during calibration

# Set to True ONLY if hexcopter translates without rotating (camera stays fixed direction)
# Set to False if drone rotates to face direction of travel (normal behavior)
USE_FIXED_HEADING = False   # Drone rotates to face movement direction
FIXED_CAMERA_HEADING = 0.0  # Only used if USE_FIXED_HEADING = True

# Earth constants
METERS_PER_DEG_LAT = 111320.0

shutdown_flag = False


class UBXGPSReader:
    """Threaded GPS reader for continuous position updates with heading"""
    def __init__(self):
        self.serial = None
        self.reader = None
        self.running = False
        self.thread = None
        self.position = None
        self.lock = threading.Lock()
        self.has_fix = False
        
    def start(self):
        print(f"[GPS] Opening {GPS_PORT}...")
        self.serial = Serial(GPS_PORT, GPS_BAUD, timeout=1)
        self.reader = UBXReader(self.serial)
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        
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
                            print(f"[GPS] ✓ FIX ({msg.numSV} sats)")
                            self.has_fix = True
                        
                        # Get heading - try headVeh first (vehicle heading), then headMot (motion heading)
                        heading = None
                        heading_valid = False
                        
                        # headVeh is vehicle heading (requires dual antenna or IMU fusion)
                        if hasattr(msg, 'headVeh'):
                            heading = msg.headVeh * 1e-5  # Scale factor for UBX
                            heading_valid = getattr(msg, 'headVehValid', False)
                        
                        # Fallback to headMot (heading of motion - only valid when moving)
                        if not heading_valid and hasattr(msg, 'headMot'):
                            heading = msg.headMot * 1e-5  # degrees
                            # headMot is valid when ground speed > ~0.5 m/s
                            gSpeed = getattr(msg, 'gSpeed', 0) / 1000.0  # mm/s to m/s
                            heading_valid = gSpeed > 0.5
                        
                        with self.lock:
                            self.position = {
                                "lat": msg.lat,
                                "lon": msg.lon,
                                "sats": msg.numSV,
                                "heading": heading if heading_valid else None,
                                "heading_raw": heading,  # Raw value even if not "valid"
                                "speed": getattr(msg, 'gSpeed', 0) / 1000.0
                            }
                    else:
                        if self.has_fix:
                            print("[GPS] ✗ FIX LOST")
                            self.has_fix = False
            except Exception as e:
                time.sleep(0.1)
                
    def get_position(self):
        with self.lock:
            return self.position.copy() if self.position else None
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()


def pixel_to_ground(px, py, H):
    """
    Convert pixel coordinates to ground offset in meters (East, North)
    The homography is calibrated so image center = (0, 0) ground
    Returns offset in CAMERA FRAME (not world frame yet)
    """
    pt = np.array([[px, py, 1.0]], dtype=np.float64).T
    g = H @ pt
    g /= g[2]
    cam_east = float(g[0])   # +X = right of camera
    cam_north = float(g[1])  # +Y = forward (direction camera points)
    return cam_east, cam_north


def camera_to_world_offset(cam_east, cam_north, current_heading, calibration_heading=CALIBRATION_HEADING):
    """
    Convert camera-frame offset to world-frame offset.
    
    cam_east: meters to the right of camera
    cam_north: meters in front of camera (direction it's pointing)
    current_heading: drone's current heading in degrees (0=North, 90=East)
    calibration_heading: heading during calibration (default North=0)
    
    Returns: (world_east, world_north) in meters
    """
    # Calculate rotation angle (how much drone has rotated from calibration)
    theta = math.radians(current_heading - calibration_heading)
    
    # Rotate from camera frame to world frame
    # Camera forward (cam_north) points in direction of current_heading
    # Camera right (cam_east) points 90° clockwise from heading
    world_east = cam_north * math.sin(theta) + cam_east * math.cos(theta)
    world_north = cam_north * math.cos(theta) - cam_east * math.sin(theta)
    
    return world_east, world_north


def ground_offset_to_gps(lat0, lon0, east_m, north_m):
    """
    Convert ground offset (meters) to GPS coordinates
    lat0, lon0: drone's current GPS position
    east_m: meters east of drone (+ve = east, -ve = west)
    north_m: meters north of drone (+ve = north, -ve = south)
    """
    # Latitude: 1 degree ≈ 111320 meters
    dlat = north_m / METERS_PER_DEG_LAT
    
    # Longitude: depends on latitude (cosine correction)
    dlon = east_m / (METERS_PER_DEG_LAT * math.cos(math.radians(lat0)))
    
    return lat0 + dlat, lon0 + dlon


def get_direction(east_m, north_m):
    """Get cardinal direction from offset"""
    angle = math.degrees(math.atan2(east_m, north_m))  # Angle from North
    if angle < 0:
        angle += 360
    
    if angle < 22.5 or angle >= 337.5:
        return "N"
    elif angle < 67.5:
        return "NE"
    elif angle < 112.5:
        return "E"
    elif angle < 157.5:
        return "SE"
    elif angle < 202.5:
        return "S"
    elif angle < 247.5:
        return "SW"
    elif angle < 292.5:
        return "W"
    else:
        return "NW"


def signal_handler(sig, frame):
    global shutdown_flag
    print("\n[STOP] Shutting down gracefully...")
    shutdown_flag = True


def main():
    global shutdown_flag
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 60)
    print("🚁 SCOUT DRONE v3 - WITH HEADING COMPENSATION")
    print("=" * 60)
    
    gps = None
    cap = None
    out = None
    csv_f = None
    frame_count = 0
    detection_count = 0
    last_valid_heading = CALIBRATION_HEADING  # Fallback to calibration heading
    heading_warning_shown = False
    
    try:
        # Load homography
        print("\n[CALIBRATION] Loading homography...")
        H = np.load(HOMOGRAPHY_PATH)
        print("[CALIBRATION] ✓ Homography loaded")
        print(f"[CALIBRATION] Calibration heading: {CALIBRATION_HEADING}° (0=North)")
        
        # Validate frame size matches calibration
        try:
            cal_size = np.load(FRAME_SIZE_PATH)
            if cal_size[0] != FRAME_W or cal_size[1] != FRAME_H:
                print(f"[WARNING] Calibration was for {cal_size[0]}x{cal_size[1]}, but using {FRAME_W}x{FRAME_H}")
                print("[WARNING] Results may be inaccurate!")
        except:
            print("[WARNING] Could not verify calibration frame size")
        
        # Verify homography - center should map to (0, 0)
        test_east, test_north = pixel_to_ground(FRAME_W/2, FRAME_H/2, H)
        print(f"[CALIBRATION] Center offset: ({test_east:.2f}, {test_north:.2f}) m")
        if abs(test_east) > 1 or abs(test_north) > 1:
            print("[WARNING] Center doesn't map to (0,0) - calibration may be off!")
        
        # Start GPS
        print("\n[GPS] Starting GPS reader...")
        gps = UBXGPSReader()
        gps.start()
        
        print("[GPS] Waiting for fix...")
        wait_start = time.time()
        while not gps.get_position():
            elapsed = int(time.time() - wait_start)
            print(f"\r[GPS] Waiting... {elapsed}s", end='', flush=True)
            time.sleep(0.5)
            if elapsed > 120:
                print("\n[GPS] TIMEOUT - No fix after 2 minutes")
                return
        
        pos = gps.get_position()
        print(f"\n[GPS] ✓ Fix acquired: {pos['lat']:.7f}, {pos['lon']:.7f} ({pos['sats']} sats)")
        if pos.get('heading') is not None:
            print(f"[GPS] ✓ Heading available: {pos['heading']:.1f}°")
        else:
            print("[GPS] ⚠ No heading from GPS - will use last known or calibration heading")
            print("[GPS] ⚠ FOR BEST ACCURACY: Fly straight paths, heading updates when moving >0.5m/s")
        
        # Open camera
        print("\n[CAMERA] Opening video device...")
        cap = cv2.VideoCapture(CAMERA_DEV, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        
        if not cap.isOpened():
            print("[CAMERA] ERROR: Could not open camera")
            return
        
        ret, _ = cap.read()
        if not ret:
            print("[CAMERA] ERROR: No frames received")
            return
        
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[CAMERA] ✓ Ready ({actual_w}x{actual_h})")
        
        if actual_w != FRAME_W or actual_h != FRAME_H:
            print(f"[WARNING] Expected {FRAME_W}x{FRAME_H}, got {actual_w}x{actual_h}")
        
        # Load YOLO model
        print("\n[MODEL] Loading YOLO...")
        model = YOLO(MODEL_PATH, task="detect")
        print("[MODEL] ✓ Loaded")
        
        # Setup video writer
        print("\n[VIDEO] Creating output file...")
        out = cv2.VideoWriter(VIDEO_PATH, cv2.VideoWriter_fourcc(*"XVID"), 10.0, (FRAME_W, FRAME_H))
        print(f"[VIDEO] ✓ {VIDEO_PATH}")
        
        # Setup CSV
        print("\n[CSV] Creating output file...")
        csv_f = open(CSV_PATH, "w", newline="")
        writer = csv.writer(csv_f)
        writer.writerow(["target_id", "latitude", "longitude", "altitude", "distance_m", "direction", "heading_deg", "timestamp"])
        print(f"[CSV] ✓ {CSV_PATH}")
        
        print("\n" + "=" * 60)
        print("🎯 READY - Flying at {}m altitude".format(ALTITUDE_M))
        print("   Heading compensation: ENABLED")
        print("   Press Ctrl+C to stop and save")
        print("=" * 60 + "\n")
        
        # Tracking state
        tracks = []
        next_id = 1
        last_ts = time.time()
        
        while not shutdown_flag:
            ret, frame = cap.read()
            if not ret:
                continue
            
            frame_count += 1
            now = time.time()
            dt = now - last_ts
            last_ts = now
            
            # Get current GPS
            pos = gps.get_position()
            if not pos:
                cv2.putText(frame, "NO GPS FIX", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                out.write(frame)
                continue
            
            lat0 = pos["lat"]
            lon0 = pos["lon"]
            
            # Determine camera heading
            if USE_FIXED_HEADING:
                # HEXCOPTER MODE: Camera always faces same direction
                camera_heading = FIXED_CAMERA_HEADING
                heading_source = "FIXED"
            else:
                # NORMAL MODE: Camera rotates with drone
                drone_heading = pos.get("heading")
                if drone_heading is not None:
                    last_valid_heading = drone_heading
                    heading_source = "GPS"
                else:
                    drone_heading = last_valid_heading
                    heading_source = "LAST" if last_valid_heading != CALIBRATION_HEADING else "CAL"
                    if not heading_warning_shown and frame_count > 100:
                        print("[WARNING] No GPS heading - using fallback.")
                        heading_warning_shown = True
                camera_heading = drone_heading
            
            # Run YOLO
            results = model(frame, conf=CONF_THRES, verbose=False)
            detections = []
            
            if results and results[0].boxes:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    conf = float(box.conf[0])
                    detections.append((cx, cy, x1, y1, x2, y2, conf))
            
            # Match detections to existing tracks
            used_tracks = set()
            for det in detections:
                cx, cy, x1, y1, x2, y2, conf = det
                
                best_track = None
                best_dist = float('inf')
                
                for i, t in enumerate(tracks):
                    if i in used_tracks:
                        continue
                    d = math.hypot(cx - t["px"][0], cy - t["px"][1])
                    if d < MATCH_RADIUS and d < best_dist:
                        best_track = i
                        best_dist = d
                
                if best_track is not None:
                    # Update existing track
                    tracks[best_track]["px"] = (cx, cy)
                    tracks[best_track]["box"] = (x1, y1, x2, y2)
                    tracks[best_track]["last_seen"] = now
                    tracks[best_track]["visible_time"] += dt
                    tracks[best_track]["conf"] = conf
                    used_tracks.add(best_track)
                else:
                    # New track
                    tracks.append({
                        "id": next_id,
                        "px": (cx, cy),
                        "box": (x1, y1, x2, y2),
                        "last_seen": now,
                        "visible_time": dt,
                        "conf": conf,
                        "logged": False,
                        "gps": None
                    })
                    next_id += 1
            
            # Process tracks - remove stale, log confirmed
            for t in list(tracks):
                # Remove if not seen for too long
                if now - t["last_seen"] > LOST_TIME:
                    tracks.remove(t)
                    continue
                
                # Log if confirmed and not already logged
                if not t["logged"] and t["visible_time"] >= CONFIRM_TIME:
                    cx, cy = t["px"]
                    
                    # Convert pixel to camera-frame offset (meters)
                    cam_east, cam_north = pixel_to_ground(cx, cy, H)
                    
                    # Rotate to world frame using camera heading
                    east_m, north_m = camera_to_world_offset(cam_east, cam_north, camera_heading)
                    
                    # Calculate distance and direction (in world frame)
                    distance = math.sqrt(east_m**2 + north_m**2)
                    direction = get_direction(east_m, north_m)
                    
                    # Convert to GPS
                    target_lat, target_lon = ground_offset_to_gps(lat0, lon0, east_m, north_m)
                    
                    # Store GPS for display
                    t["gps"] = (target_lat, target_lon)
                    t["offset"] = (east_m, north_m)
                    t["distance"] = distance
                    t["direction"] = direction
                    t["heading_at_lock"] = camera_heading
                    
                    # Log to CSV
                    timestamp = time.strftime("%H:%M:%S")
                    writer.writerow([
                        t["id"],
                        f"{target_lat:.7f}",
                        f"{target_lon:.7f}",
                        ALTITUDE_M,
                        f"{distance:.1f}",
                        direction,
                        f"{camera_heading:.1f}",
                        timestamp
                    ])
                    csv_f.flush()
                    
                    t["logged"] = True
                    detection_count += 1
                    
                    print(f"🎯 TARGET #{detection_count} (ID:{t['id']})")
                    print(f"   Pixel: ({cx:.0f}, {cy:.0f})")
                    print(f"   Camera offset: ({cam_east:+.1f}, {cam_north:+.1f}) m")
                    print(f"   Camera HDG: {camera_heading:.1f}° → World offset: ({east_m:+.1f}E, {north_m:+.1f}N) m")
                    print(f"   Distance: {distance:.1f}m {direction}")
                    print(f"   GPS: {target_lat:.7f}, {target_lon:.7f}")
                    print()
            
            # Draw on frame
            for t in tracks:
                x1, y1, x2, y2 = [int(v) for v in t["box"]]
                cx, cy = int(t["px"][0]), int(t["px"][1])
                
                if t["logged"]:
                    color = (0, 0, 255)  # Red = confirmed
                    thickness = 3
                elif t["visible_time"] >= CONFIRM_TIME * 0.5:
                    color = (0, 165, 255)  # Orange = almost confirmed
                    thickness = 2
                else:
                    color = (0, 255, 0)  # Green = tracking
                    thickness = 2
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                
                # Draw center dot
                cv2.circle(frame, (cx, cy), 4, color, -1)
                
                # Label
                label = f"ID:{t['id']} {t['visible_time']:.1f}s"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                if t["logged"] and t.get("distance"):
                    info = f"{t['distance']:.1f}m {t['direction']}"
                    cv2.putText(frame, info, (x1, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.putText(frame, "LOCKED", (x1, y2 + 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Draw frame center crosshair (drone position reference)
            fcx, fcy = FRAME_W // 2, FRAME_H // 2
            cv2.line(frame, (fcx - 20, fcy), (fcx + 20, fcy), (255, 255, 0), 1)
            cv2.line(frame, (fcx, fcy - 20), (fcx, fcy + 20), (255, 255, 0), 1)
            
            # Draw heading indicator (arrow pointing direction camera faces)
            arrow_len = 40
            arrow_end_x = int(fcx + arrow_len * math.sin(math.radians(camera_heading)))
            arrow_end_y = int(fcy - arrow_len * math.cos(math.radians(camera_heading)))
            cv2.arrowedLine(frame, (fcx, fcy), (arrow_end_x, arrow_end_y), (0, 255, 255), 2, tipLength=0.3)
            
            # Status bar with heading
            heading_color = (0, 255, 0) if heading_source == "GPS" else (0, 165, 255)
            status = f"F:{frame_count} | T:{detection_count} | GPS:{pos['sats']}sat"
            cv2.putText(frame, status, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"{lat0:.6f}, {lon0:.6f}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(frame, f"CAM: {camera_heading:.0f}° [{heading_source}]", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, heading_color, 2)
            
            out.write(frame)
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n" + "=" * 60)
        print("SHUTDOWN")
        print("=" * 60)
        
        if cap:
            cap.release()
            print("[CAMERA] ✓ Released")
        
        if out:
            out.release()
            print(f"[VIDEO] ✓ Saved {VIDEO_PATH}")
        
        if gps:
            gps.stop()
            print("[GPS] ✓ Stopped")
        
        if csv_f:
            csv_f.close()
            print(f"[CSV] ✓ Saved {CSV_PATH}")
        
        print(f"\n📊 Summary: {frame_count} frames, {detection_count} targets")
        print("=" * 60)


if __name__ == "__main__":
    main()
