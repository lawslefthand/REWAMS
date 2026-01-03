import cv2
import math
import time
import threading
from ultralytics import YOLO
from serial import Serial
from pyubx2 import UBXReader

# Configuration
MODEL_PATH = "/home/aryan/Desktop/best_ncnn_model"
CAMERA_DEV = "/dev/video0"
FRAME_W = 640
FRAME_H = 480
ALTITUDE_M = 20.0
CONF_THRES = 0.45

# Test different FOV values
FOV_TEST_VALUES = [90.0, 130.0, 150.0, 170.0]
current_fov_idx = 0
FOV_H_DEG = FOV_TEST_VALUES[current_fov_idx]

# Known ground truth - UPDATE THESE WITH YOUR MEASURED VALUES
MARKER_DISTANCE_M = 10.0  # Distance between your two humans in meters

# Globals
cap = None
gps = None
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
        print(f"[GPS] Starting on {self.port}...")
        self.serial = Serial(self.port, self.baudrate, timeout=1)
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
                            print(f"[GPS] ✓ FIX ACQUIRED - {msg.numSV} satellites")
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
                            print("[GPS] ✗ FIX LOST")
                            self.has_fix = False
            except Exception as e:
                if self.running:
                    print(f"[GPS] Error: {e}")
                time.sleep(0.1)
                
    def get_position(self):
        with self.lock:
            return self.position
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()

def bbox_to_ground(cx, cy, fov_deg):
    """Convert pixel coords to ground offset (North, East) in meters"""
    dx = cx - FRAME_W / 2
    dy = cy - FRAME_H / 2
    fov_h = math.radians(fov_deg)
    ground_w = 2 * ALTITUDE_M * math.tan(fov_h / 2)
    fov_v = 2 * math.atan((FRAME_H / FRAME_W) * math.tan(fov_h / 2))
    ground_h = 2 * ALTITUDE_M * math.tan(fov_v / 2)
    mx = ground_w / FRAME_W
    my = ground_h / FRAME_H
    return -dy * my, dx * mx

def project(lat0, lon0, north, east):
    """Project ground offset to GPS coordinates"""
    dlat = north / 111320.0
    dlon = east / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS points in meters (Haversine)"""
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    
    a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def calculate_fov_from_markers(pixel_distance):
    """Calculate FOV from known marker distance and pixel measurement"""
    meters_per_pixel = MARKER_DISTANCE_M / pixel_distance
    ground_width = meters_per_pixel * FRAME_W
    fov_rad = 2 * math.atan((ground_width / 2) / ALTITUDE_M)
    return math.degrees(fov_rad)

def draw_overlay(frame, detections, tracks, drone_pos, fov_deg):
    """Draw detection boxes, IDs, crosshairs, and info overlay"""
    overlay = frame.copy()
    
    # Draw center crosshairs
    cv2.line(overlay, (FRAME_W//2, 0), (FRAME_W//2, FRAME_H), (0, 255, 0), 1)
    cv2.line(overlay, (0, FRAME_H//2), (FRAME_W, FRAME_H//2), (0, 255, 0), 1)
    cv2.circle(overlay, (FRAME_W//2, FRAME_H//2), 5, (0, 255, 0), -1)
    
    # Draw detection boxes and IDs
    for i, det in enumerate(detections):
        x1, y1, x2, y2, conf, cx, cy = det
        
        # Find if this detection matches a track
        track_id = None
        for t in tracks:
            if abs(t["px"][0] - cx) < 50 and abs(t["px"][1] - cy) < 50:
                track_id = t["id"]
                break
        
        # Color based on tracking status
        if track_id:
            color = (0, 255, 0)  # Green for tracked
            label = f"ID:{track_id} {conf:.2f}"
        else:
            color = (255, 255, 0)  # Cyan for new detection
            label = f"NEW {conf:.2f}"
        
        # Draw box
        cv2.rectangle(overlay, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        
        # Draw label background
        (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(overlay, (int(x1), int(y1)-label_h-5), (int(x1)+label_w, int(y1)), color, -1)
        cv2.putText(overlay, label, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Draw center point
        cv2.circle(overlay, (int(cx), int(cy)), 3, (0, 0, 255), -1)
        
        # Calculate and show offset
        offset_x = cx - FRAME_W/2
        offset_y = cy - FRAME_H/2
        north, east = bbox_to_ground(cx, cy, fov_deg)
        
        offset_text = f"({offset_x:+.0f}px, {offset_y:+.0f}px) = ({north:+.1f}m N, {east:+.1f}m E)"
        cv2.putText(overlay, offset_text, (int(x1), int(y2)+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    
    # Info panel (top-left)
    y_offset = 20
    info_lines = [
        f"FOV: {fov_deg:.1f}° (Press N for next)",
        f"Detections: {len(detections)}",
        f"Tracked: {len([t for t in tracks if not t['done']])}",
    ]
    
    if drone_pos:
        ground_w = 2 * ALTITUDE_M * math.tan(math.radians(fov_deg) / 2)
        info_lines.append(f"Ground: {ground_w:.1f}m x {ground_w * FRAME_H/FRAME_W:.1f}m")
        info_lines.append(f"GPS: {drone_pos['latitude']:.7f}, {drone_pos['longitude']:.7f}")
        info_lines.append(f"Sats: {drone_pos['numSV']}")
    
    for line in info_lines:
        cv2.putText(overlay, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        y_offset += 20
    
    # Instructions (bottom)
    instructions = [
        "Q: Quit | N: Next FOV | SPACE: Calculate FOV from markers",
        "C: Clear tracks | S: Save frame"
    ]
    y_pos = FRAME_H - 35
    for line in instructions:
        cv2.putText(overlay, line, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        y_pos += 15
    
    return overlay

def main():
    global shutdown_flag, cap, gps, FOV_H_DEG, current_fov_idx
    
    print("=" * 70)
    print("FOV CALIBRATION & DETECTION TEST")
    print("=" * 70)
    print(f"Marker distance: {MARKER_DISTANCE_M}m")
    print(f"Altitude: {ALTITUDE_M}m")
    print(f"Starting FOV: {FOV_H_DEG}°")
    print("=" * 70)
    
    try:
        # Load model
        print("\n[MODEL] Loading YOLO...")
        model = YOLO(MODEL_PATH, task="detect")
        print("[MODEL] ✓ Loaded")
        
        # Initialize camera
        print("\n[CAMERA] Initializing...")
        cap = cv2.VideoCapture(CAMERA_DEV, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        
        if not cap.isOpened():
            raise RuntimeError("Camera failed to open")
        print("[CAMERA] ✓ Ready")
        
        # Initialize GPS
        print("\n[GPS] Starting...")
        gps = UBXGPSReader("/dev/ttyAMA0", 230400)
        gps.start()
        
        # Wait for GPS fix
        print("[GPS] Waiting for fix...")
        while not gps.get_position():
            time.sleep(0.5)
        
        pos = gps.get_position()
        print(f"[GPS] ✓ Fix: {pos['latitude']:.7f}, {pos['longitude']:.7f}")
        print(f"[GPS] Satellites: {pos['numSV']}")
        
        print("\n" + "=" * 70)
        print("READY! Fly to 20m and hover over markers")
        print("=" * 70)
        
        tracks = []
        next_id = 1
        last_ts = time.time()
        
        cv2.namedWindow("FOV Calibration", cv2.WINDOW_NORMAL)
        
        detected_coords = []  # Store coordinates of detected humans
        
        while not shutdown_flag:
            ret, frame = cap.read()
            if not ret:
                continue
            
            pos = gps.get_position()
            if not pos:
                continue
            
            lat0 = pos["latitude"]
            lon0 = pos["longitude"]
            
            now = time.time()
            dt = now - last_ts
            last_ts = now
            
            # Run detection
            results = model(frame, conf=CONF_THRES, verbose=False)
            detections = []
            if results and results[0].boxes:
                for b in results[0].boxes:
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    conf = b.conf[0].item()
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    detections.append((x1, y1, x2, y2, conf, cx, cy))
            
            # Update tracks
            used = set()
            for det in detections:
                x1, y1, x2, y2, conf, cx, cy = det
                
                best = None
                best_d = 1e9
                for i, t in enumerate(tracks):
                    if i in used:
                        continue
                    d = math.hypot(cx - t["px"][0], cy - t["px"][1])
                    if d < 120 and d < best_d:
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
                    print(f"\n🆕 NEW TRACK ID:{next_id} at ({cx:.0f}, {cy:.0f})")
                    next_id += 1
            
            # Remove lost tracks
            for t in list(tracks):
                if now - t["last"] > 4.0:
                    print(f"❌ LOST TRACK ID:{t['id']}")
                    tracks.remove(t)
                    continue
                
                # Confirm tracks and calculate coordinates
                if not t["done"] and t["time"] >= 2.0:
                    north, east = bbox_to_ground(t["px"][0], t["px"][1], FOV_H_DEG)
                    lat, lon = project(lat0, lon0, north, east)
                    t["done"] = True
                    t["gps"] = (lat, lon)
                    detected_coords.append((lat, lon, t["id"]))
                    
                    print(f"\n✅ CONFIRMED ID:{t['id']}")
                    print(f"   Pixel: ({t['px'][0]:.0f}, {t['px'][1]:.0f})")
                    print(f"   Offset: N={north:.1f}m, E={east:.1f}m")
                    print(f"   GPS: {lat:.7f}, {lon:.7f}")
            
            # Draw overlay
            display = draw_overlay(frame, detections, tracks, pos, FOV_H_DEG)
            cv2.imshow("FOV Calibration", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n[QUIT] Exiting...")
                break
            
            elif key == ord('n'):
                # Cycle through FOV values
                current_fov_idx = (current_fov_idx + 1) % len(FOV_TEST_VALUES)
                FOV_H_DEG = FOV_TEST_VALUES[current_fov_idx]
                print(f"\n📐 FOV changed to {FOV_H_DEG}°")
                
                # Recalculate all confirmed track coordinates
                for t in tracks:
                    if t["done"]:
                        north, east = bbox_to_ground(t["px"][0], t["px"][1], FOV_H_DEG)
                        lat, lon = project(lat0, lon0, north, east)
                        t["gps"] = (lat, lon)
                        print(f"   ID:{t['id']} → {lat:.7f}, {lon:.7f}")
            
            elif key == ord('c'):
                # Clear all tracks
                tracks.clear()
                detected_coords.clear()
                next_id = 1
                print("\n🗑️  Tracks cleared")
            
            elif key == ord('s'):
                # Save current frame
                filename = f"calibration_{int(time.time())}.jpg"
                cv2.imwrite(filename, display)
                print(f"\n💾 Saved: {filename}")
            
            elif key == ord(' '):
                # Calculate FOV from two detected markers
                if len(detected_coords) >= 2:
                    # Use last two detections
                    lat1, lon1, id1 = detected_coords[-2]
                    lat2, lon2, id2 = detected_coords[-1]
                    
                    gps_distance = calculate_distance(lat1, lon1, lat2, lon2)
                    
                    # Get pixel distance
                    t1 = next((t for t in tracks if t["id"] == id1), None)
                    t2 = next((t for t in tracks if t["id"] == id2), None)
                    
                    if t1 and t2:
                        pixel_dist = math.hypot(
                            t2["px"][0] - t1["px"][0],
                            t2["px"][1] - t1["px"][1]
                        )
                        
                        calculated_fov = calculate_fov_from_markers(pixel_dist)
                        
                        print("\n" + "=" * 70)
                        print("FOV CALCULATION FROM MARKERS")
                        print("=" * 70)
                        print(f"Marker distance (known): {MARKER_DISTANCE_M:.1f}m")
                        print(f"GPS distance (measured): {gps_distance:.1f}m")
                        print(f"Pixel distance: {pixel_dist:.1f}px")
                        print(f"Calculated FOV: {calculated_fov:.1f}°")
                        print(f"Current FOV setting: {FOV_H_DEG:.1f}°")
                        print(f"Error: {abs(gps_distance - MARKER_DISTANCE_M):.1f}m")
                        print("=" * 70)
                        
                        if abs(gps_distance - MARKER_DISTANCE_M) < 2.0:
                            print(f"✅ Good accuracy! Use FOV = {calculated_fov:.1f}° in your script")
                        else:
                            print(f"⚠️  Large error - ensure markers are exactly {MARKER_DISTANCE_M}m apart")
                    else:
                        print("❌ Both tracks need to be active")
                else:
                    print("❌ Need at least 2 confirmed detections")
        
        print("\n" + "=" * 70)
        print("FINAL RESULTS")
        print("=" * 70)
        print(f"Total detections: {len(detected_coords)}")
        for lat, lon, track_id in detected_coords:
            print(f"  ID:{track_id} → {lat:.7f}, {lon:.7f}")
        
        if len(detected_coords) >= 2:
            lat1, lon1, _ = detected_coords[0]
            lat2, lon2, _ = detected_coords[-1]
            final_distance = calculate_distance(lat1, lon1, lat2, lon2)
            error = abs(final_distance - MARKER_DISTANCE_M)
            print(f"\nDistance between first and last: {final_distance:.1f}m")
            print(f"Expected: {MARKER_DISTANCE_M}m")
            print(f"Error: {error:.1f}m ({error/MARKER_DISTANCE_M*100:.1f}%)")
        
        print("=" * 70)
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if cap:
            cap.release()
        if gps:
            gps.stop()
        cv2.destroyAllWindows()
        print("\n[DONE] Cleanup complete")

if __name__ == "__main__":
    main()
