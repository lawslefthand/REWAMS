import cv2
import math
import time
import csv
import signal
import subprocess
import numpy as np
from ultralytics import YOLO
from serial import Serial
from pyubx2 import UBXReader
import threading
import lgpio

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
CSV_DELIVERY = "reload_rearm.csv"      # For delivery drone (lat, lon, alt)
CSV_DETAILED = "targets_detailed.csv"  # Full details with IDs

BUZZER_PIN = 17
BUZZER_DURATION = 2.0  # seconds to buzz when target locked

# ===== MISSION TIMER =====
MISSION_DURATION = 12 * 60  # 12 minutes in seconds

# ===== DELIVERY DRONE SCP =====
DELIVERY_DRONE_TARGET = "aryan@10.131.49.10:/home/aryan/"
SCP_RETRY_INTERVAL = 10  # seconds between retries
SCP_MAX_RETRIES = 30     # Max retries (5 minutes worth)

# ===== HEADING CONFIGURATION =====
# The heading (degrees) the camera was pointing during calibration
# 0 = North, 90 = East, 180 = South, 270 = West
CALIBRATION_HEADING = 0.0  # Camera was pointing NORTH during calibration

# Set to True ONLY if hexcopter translates without rotating (camera stays fixed direction)
# Set to False if drone rotates to face direction of travel (normal behavior)
USE_FIXED_HEADING = False   # Drone rotates to face movement direction
FIXED_CAMERA_HEADING = 0.0  # Only used if USE_FIXED_HEADING = True

# ===== CALIBRATION OFFSET CORRECTION =====
# If drone wasn't centered during calibration, the image center won't map to (0,0)
# Enter the values shown by calibrate_v2.py: "Image center maps to ground: (X, Y) meters"
# These will be subtracted to correct the offset
CALIBRATION_OFFSET_EAST = 3.08   # meters (from calibration output)
CALIBRATION_OFFSET_NORTH = 0.56  # meters (from calibration output)

# Earth constants
METERS_PER_DEG_LAT = 111320.0

# Deduplication threshold (don't log same human twice)
DEDUP_DISTANCE_M = 4.0

# Heading stability threshold (reject logging during turns)
HEADING_JUMP_THRESHOLD = 8.0  # degrees

shutdown_flag = False
mission_start_time = None
logged_targets = []  # List of (lat, lon) for deduplication


def send_csv_to_delivery_drone():
    """Send reload_rearm.csv to delivery drone via SCP"""
    print("\n" + "=" * 60)
    print("📡 SENDING COORDINATES TO DELIVERY DRONE")
    print("=" * 60)
    print(f"Target: {DELIVERY_DRONE_TARGET}")
    print(f"File: {CSV_DELIVERY}")
    print(f"Retry interval: {SCP_RETRY_INTERVAL}s")
    print()
    
    for attempt in range(1, SCP_MAX_RETRIES + 1):
        print(f"[SCP] Attempt {attempt}/{SCP_MAX_RETRIES}...")
        
        result = subprocess.run(
            ["scp", CSV_DELIVERY, DELIVERY_DRONE_TARGET],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE
        )
        
        if result.returncode == 0:
            print(f"[SCP] ✓ CSV sent successfully to delivery drone!")
            return True
        else:
            error = result.stderr.decode().strip() if result.stderr else "Unknown error"
            print(f"[SCP] ✗ Failed: {error}")
            if attempt < SCP_MAX_RETRIES:
                print(f"[SCP] Retrying in {SCP_RETRY_INTERVAL}s...")
                time.sleep(SCP_RETRY_INTERVAL)
    
    print(f"[SCP] ✗ Failed after {SCP_MAX_RETRIES} attempts")
    return False


class Buzzer:
    """GPIO buzzer controller (active-low)"""
    def __init__(self, pin):
        self.pin = pin
        self.h = lgpio.gpiochip_open(4)  # Pi 5 uses gpiochip4
        lgpio.gpio_claim_output(self.h, pin, 1)  # Start OFF (HIGH)
        self.buzzing = False
        self.buzz_end = 0
    
    def on(self):
        lgpio.gpio_write(self.h, self.pin, 0)  # Active-low: LOW = ON
        self.buzzing = True
    
    def off(self):
        lgpio.gpio_write(self.h, self.pin, 1)  # HIGH = OFF
        self.buzzing = False
    
    def buzz_for(self, duration):
        """Start buzzing for specified duration"""
        self.on()
        self.buzz_end = time.time() + duration
    
    def update(self):
        """Call in main loop to auto-stop after duration"""
        if self.buzzing and time.time() >= self.buzz_end:
            self.off()
    
    def cleanup(self):
        self.off()
        lgpio.gpiochip_close(self.h)


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


def pixel_to_ground(px, py, H, frame_w=FRAME_W, frame_h=FRAME_H):
    """
    Convert pixel coordinates to ground offset in meters (East, North)
    The homography is calibrated so image center = (0, 0) ground
    Returns offset in CAMERA FRAME (not world frame yet)
    """
    # Scale pixel coordinates if frame size differs from calibration
    try:
        cal_size = np.load(FRAME_SIZE_PATH)
        cal_w, cal_h = cal_size[0], cal_size[1]
        if cal_w != frame_w or cal_h != frame_h:
            # Scale to calibration image coordinates
            px = px * cal_w / frame_w
            py = py * cal_h / frame_h
    except:
        pass  # Use original coordinates if can't load calibration size
    
    pt = np.array([[px, py, 1.0]], dtype=np.float64).T
    g = H @ pt
    g /= g[2, 0]
    
    # Apply offset correction (if drone wasn't centered during calibration)
    cam_east = float(g[0, 0]) - CALIBRATION_OFFSET_EAST
    cam_north = float(g[1, 0]) - CALIBRATION_OFFSET_NORTH
    
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


def is_duplicate_target(lat, lon, threshold=DEDUP_DISTANCE_M):
    """Check if this target is too close to an already logged target"""
    global logged_targets
    for lat2, lon2 in logged_targets:
        # Calculate distance between two GPS points
        d_north = (lat - lat2) * METERS_PER_DEG_LAT
        d_east = (lon - lon2) * METERS_PER_DEG_LAT * math.cos(math.radians(lat))
        dist = math.hypot(d_east, d_north)
        if dist < threshold:
            return True
    return False


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
    out = None  # Video writer
    csv_delivery = None
    csv_detailed = None
    buzzer = None
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
                print(f"[CALIBRATION] Calibration image: {cal_size[0]}x{cal_size[1]}, Camera: {FRAME_W}x{FRAME_H}")
                print("[CALIBRATION] Auto-scaling pixel coordinates to match")
        except:
            print("[WARNING] Could not verify calibration frame size")
        
        # Verify homography - center should map to (0, 0) after correction
        test_east, test_north = pixel_to_ground(FRAME_W/2, FRAME_H/2, H)
        print(f"[CALIBRATION] Center offset (corrected): ({test_east:.2f}, {test_north:.2f}) m")
        print(f"[CALIBRATION] Offset correction applied: ({CALIBRATION_OFFSET_EAST}, {CALIBRATION_OFFSET_NORTH}) m")
        if abs(test_east) > 1 or abs(test_north) > 1:
            print("[WARNING] Center doesn't map to (0,0) - check CALIBRATION_OFFSET values!")
        
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
            # Auto-set calibration heading to current heading at startup
            if CALIBRATION_HEADING == 0.0:
                global CALIBRATION_HEADING
                CALIBRATION_HEADING = pos['heading']
                print(f"[GPS] ✓ Auto-set calibration heading to {CALIBRATION_HEADING:.1f}°")
        else:
            print("[GPS] ⚠ No heading from GPS - will use last known or calibration heading")
            print("[GPS] ⚠ FOR BEST ACCURACY: Fly straight paths, heading updates when moving >0.5m/s")
        
        # Initialize buzzer
        print("\n[BUZZER] Initializing GPIO 17...")
        buzzer = Buzzer(BUZZER_PIN)
        print("[BUZZER] ✓ Ready")
        
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
        
        # Setup CSVs
        print("\n[CSV] Creating output files...")
        csv_delivery = open(CSV_DELIVERY, "w", newline="")
        writer_delivery = csv.writer(csv_delivery)
        writer_delivery.writerow(["lat", "lon", "alt"])
        print(f"[CSV] ✓ {CSV_DELIVERY} (for delivery drone)")
        
        csv_detailed = open(CSV_DETAILED, "w", newline="")
        writer_detailed = csv.writer(csv_detailed)
        writer_detailed.writerow(["target_id", "latitude", "longitude", "altitude", "distance_m", "direction", "heading_deg", "timestamp"])
        print(f"[CSV] ✓ {CSV_DETAILED} (with IDs)")
        
        # Setup video writer (write frames in real-time to save memory)
        print("\n[VIDEO] Creating output file...")
        out = cv2.VideoWriter(VIDEO_PATH, cv2.VideoWriter_fourcc(*"XVID"), 10.0, (FRAME_W, FRAME_H))
        print(f"[VIDEO] ✓ {VIDEO_PATH}")
        
        # Start mission timer
        global mission_start_time
        mission_start_time = time.time()
        mission_end_time = mission_start_time + MISSION_DURATION
        
        print("\n" + "=" * 60)
        print("🎯 READY - HEADLESS MODE at {}m altitude".format(ALTITUDE_M))
        print("   Heading compensation: ENABLED")
        print("   Buzzer: 2s on target lock")
        print(f"   ⏱️  Mission timer: {MISSION_DURATION//60} minutes")
        print("   Press Ctrl+C to stop early")
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
            
            # Check mission timer (12 minutes)
            elapsed = now - mission_start_time
            remaining = MISSION_DURATION - elapsed
            if remaining <= 0:
                print("\n⏱️  MISSION TIME COMPLETE (12 minutes)")
                break
            
            # Show timer every 60 seconds
            if frame_count % 600 == 0:  # ~60s at 10fps
                mins = int(remaining // 60)
                secs = int(remaining % 60)
                print(f"[TIMER] {mins}:{secs:02d} remaining")
            
            # Update buzzer (auto-stop after duration)
            if buzzer:
                buzzer.update()
            
            # Get current GPS
            pos = gps.get_position()
            if not pos:
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
            
            # Get annotated frame from YOLO (with boxes, labels, etc.)
            annotated_frame = results[0].plot() if results else frame.copy()
            
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
                    # Update existing track with running average for static targets
                    t = tracks[best_track]
                    # Exponential moving average for smoother position (static humans)
                    alpha = 0.3  # Smoothing factor
                    old_cx, old_cy = t["px"]
                    new_cx = alpha * cx + (1 - alpha) * old_cx
                    new_cy = alpha * cy + (1 - alpha) * old_cy
                    
                    t["px"] = (new_cx, new_cy)
                    t["box"] = (x1, y1, x2, y2)
                    t["last_seen"] = now
                    t["visible_time"] += dt
                    t["conf"] = conf
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
                
                # Capture GPS reference when track is ~80% confirmed (lock GPS early)
                if t["visible_time"] >= CONFIRM_TIME * 0.8 and "gps_ref" not in t:
                    t["gps_ref"] = (lat0, lon0)
                    t["heading_ref"] = camera_heading
                
                # Log if confirmed and not already logged
                if not t["logged"] and t["visible_time"] >= CONFIRM_TIME:
                    
                    # GUARD 1: Reject if heading jumped too much (turn in progress)
                    heading_jump = abs(camera_heading - last_valid_heading)
                    if heading_jump > 180:
                        heading_jump = 360 - heading_jump  # Handle wraparound
                    if heading_jump > HEADING_JUMP_THRESHOLD:
                        continue  # Skip this frame, wait for stable heading
                    
                    cx, cy = t["px"]
                    
                    # Use locked GPS reference if available, else current
                    ref_lat, ref_lon = t.get("gps_ref", (lat0, lon0))
                    ref_heading = t.get("heading_ref", camera_heading)
                    
                    # Convert pixel to camera-frame offset (meters)
                    cam_east, cam_north = pixel_to_ground(cx, cy, H)
                    
                    # Rotate to world frame using reference heading
                    east_m, north_m = camera_to_world_offset(cam_east, cam_north, ref_heading)
                    
                    # Calculate distance and direction (in world frame)
                    distance = math.sqrt(east_m**2 + north_m**2)
                    direction = get_direction(east_m, north_m)
                    
                    # Convert to GPS using reference position
                    target_lat, target_lon = ground_offset_to_gps(ref_lat, ref_lon, east_m, north_m)
                    
                    # GUARD 2: Check for duplicate (same human logged before)
                    if is_duplicate_target(target_lat, target_lon):
                        t["logged"] = True  # Mark as logged to stop retrying
                        print(f"[SKIP] ID:{t['id']} - duplicate of existing target")
                        continue
                    
                    # Store GPS for display
                    t["gps"] = (target_lat, target_lon)
                    t["offset"] = (east_m, north_m)
                    t["distance"] = distance
                    t["direction"] = direction
                    t["heading_at_lock"] = ref_heading
                    
                    # Add to logged targets for deduplication
                    logged_targets.append((target_lat, target_lon))
                    
                    # Log to delivery CSV (lat, lon, alt)
                    writer_delivery.writerow([
                        f"{target_lat:.7f}",
                        f"{target_lon:.7f}",
                        20  # Fixed altitude
                    ])
                    csv_delivery.flush()
                    
                    # Log to detailed CSV
                    timestamp = time.strftime("%H:%M:%S")
                    writer_detailed.writerow([
                        t["id"],
                        f"{target_lat:.7f}",
                        f"{target_lon:.7f}",
                        20,
                        f"{distance:.1f}",
                        direction,
                        f"{ref_heading:.1f}",
                        timestamp
                    ])
                    csv_detailed.flush()
                    
                    t["logged"] = True
                    detection_count += 1
                    
                    # BUZZ for 2 seconds on target lock
                    if buzzer:
                        buzzer.buzz_for(BUZZER_DURATION)
                    
                    print(f"🎯 TARGET #{detection_count} (ID:{t['id']}) 🔊 BUZZING")
                    print(f"   Pixel: ({cx:.0f}, {cy:.0f})")
                    print(f"   Camera offset: ({cam_east:+.1f}, {cam_north:+.1f}) m")
                    print(f"   Camera HDG: {camera_heading:.1f}° → World offset: ({east_m:+.1f}E, {north_m:+.1f}N) m")
                    print(f"   Distance: {distance:.1f}m {direction}")
                    print(f"   GPS: {target_lat:.7f}, {target_lon:.7f}")
                    print()
            
            # Add tracking info overlay on YOLO annotated frame
            for t in tracks:
                x1, y1, x2, y2 = [int(v) for v in t["box"]]
                
                # Add our tracking ID and status on top of YOLO boxes
                if t["logged"]:
                    label = f"ID:{t['id']} LOCKED"
                    color = (0, 0, 255)  # Red
                    if t.get("distance"):
                        info = f"{t['distance']:.1f}m {t['direction']}"
                        cv2.putText(annotated_frame, info, (x1, y2 + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                else:
                    label = f"ID:{t['id']} {t['visible_time']:.1f}s"
                    color = (0, 255, 0)  # Green
                
                cv2.putText(annotated_frame, label, (x1, y1 - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Status bar
            heading_color = (0, 255, 0) if heading_source == "GPS" else (0, 165, 255)
            status = f"F:{frame_count} | T:{detection_count} | GPS:{pos['sats']}sat | HDG:{camera_heading:.0f}°"
            cv2.putText(annotated_frame, status, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated_frame, f"{lat0:.6f}, {lon0:.6f}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # Write frame to video (real-time to save memory)
            out.write(annotated_frame)
            
            # Progress indicator every 100 frames
            if frame_count % 100 == 0:
                print(f"[RUNNING] Frame {frame_count} | Targets: {detection_count}")
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n" + "=" * 60)
        print("SHUTDOWN - Saving outputs...")
        print("=" * 60)
        
        if buzzer:
            buzzer.cleanup()
            print("[BUZZER] ✓ Cleaned up")
        
        if cap:
            cap.release()
            print("[CAMERA] ✓ Released")
        
        if gps:
            gps.stop()
            print("[GPS] ✓ Stopped")
        
        if csv_delivery:
            csv_delivery.close()
            print(f"[CSV] ✓ Saved {CSV_DELIVERY}")
        
        if csv_detailed:
            csv_detailed.close()
            print(f"[CSV] ✓ Saved {CSV_DETAILED}")
        
        if out:
            out.release()
            print(f"[VIDEO] ✓ Saved {VIDEO_PATH}")
        
        print(f"\n📊 Summary: {frame_count} frames, {detection_count} targets")
        print(f"📁 Files saved:")
        print(f"   - {VIDEO_PATH} (annotated video)")
        print(f"   - {CSV_DELIVERY} (lat, lon, alt for delivery drone)")
        print(f"   - {CSV_DETAILED} (full details with IDs)")
        print("=" * 60)
        
        # Send coordinates to delivery drone
        if detection_count > 0:
            send_csv_to_delivery_drone()
        else:
            print("\n[SCP] No targets detected - skipping delivery drone upload")


if __name__ == "__main__":
    main()
