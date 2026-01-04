from pymavlink import mavutil
import time
import math
import csv

# ===============================
# CONFIG
# ===============================
CSV_PATH = "/home/danba/reload_rearm.csv"

DEFAULT_ALT = 20.0        # meters
DROP_ALT = 5.0            # meters
HOVER_TIME = 5            # seconds

ARRIVAL_THRESH = 2.0      # meters
ALT_TOL = 0.8             # meters (REAL WORLD)
DESCENT_TIMEOUT = 20      # seconds
POS_TIMEOUT = 1.0         # seconds

GROUND_WAIT = 40          # seconds

SERVO_NUM = 9             # AUX1 = SERVO9
SERVO_OPEN = 2000
SERVO_CLOSE = 1000

# ===============================
# CSV LOADER
# ===============================
def load_csv_waypoints(path, default_alt):
    wps = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            lat = float(row["lat"])
            lon = float(row["lon"])
            alt = float(row.get("alt", default_alt))
            wps.append((lat, lon, alt))
    return wps

# ===============================
# CONNECT
# ===============================
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
master.wait_heartbeat()
print("Connected to vehicle")

# ===============================
# HELPERS
# ===============================
def get_pos():
    msg = master.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=POS_TIMEOUT
    )
    if msg is None:
        return None
    return (
        msg.lat / 1e7,
        msg.lon / 1e7,
        msg.relative_alt / 1000.0
    )

def distance_m(lat1, lon1, lat2, lon2):
    R = 6371000
    x = math.radians(lat2 - lat1)
    y = math.radians(lon2 - lon1)
    a = (
        math.sin(x / 2) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(y / 2) ** 2
    )
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def set_servo(pwm, label=""):
    print(f"Servo {label} | PWM = {pwm}")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        SERVO_NUM,
        pwm,
        0, 0, 0, 0, 0
    )

def arm_and_takeoff(alt):
    print("Closing servo before takeoff")
    set_servo(SERVO_CLOSE, "CLOSE")
    time.sleep(5)

    master.set_mode_apm("GUIDED")
    time.sleep(1)

    master.arducopter_arm()
    time.sleep(3)

    print(f"Taking off to {alt} m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, alt
    )

    time.sleep(8)

def wait_until_landed():
    print("Waiting for landing")
    while True:
        pos = get_pos()
        if pos is None:
            continue
        _, _, alt = pos
        if alt < 0.15:
            print("Landed")
            break
        time.sleep(0.5)

def goto(lat, lon, alt):
    print(f"Going to {lat}, {lon}, {alt}")
    master.set_mode_apm("GUIDED")
    time.sleep(0.5)

    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    while True:
        pos = get_pos()
        if pos is None:
            continue

        clat, clon, _ = pos
        d = distance_m(clat, clon, lat, lon)
        print(f"  Distance: {d:.2f} m", end="\r")

        if d < ARRIVAL_THRESH:
            print("\nReached waypoint")
            break

        time.sleep(0.3)

def descend_and_hover(target_alt, hover_time):
    print(f"Descending to {target_alt} m")
    master.set_mode_apm("GUIDED")
    time.sleep(0.5)

    pos = get_pos()
    if pos is None:
        print("No position data, skipping descent")
        return

    lat, lon, _ = pos

    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        target_alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    start = time.time()
    while time.time() - start < DESCENT_TIMEOUT:
        pos = get_pos()
        if pos is None:
            continue
        _, _, alt = pos
        if abs(alt - target_alt) < ALT_TOL:
            print(f"Reached {target_alt} m")
            break
        time.sleep(0.3)
    else:
        print("Descent timeout, continuing anyway")

    print(f"Hovering for {hover_time} seconds")
    time.sleep(hover_time)

# ===============================
# MAIN
# ===============================
waypoints = load_csv_waypoints(CSV_PATH, DEFAULT_ALT)
if not waypoints:
    raise RuntimeError("CSV has no waypoints")

print(f"Loaded {len(waypoints)} waypoints")

# ---- MISSION TIMER START ----
mission_start_time = time.time()

arm_and_takeoff(DEFAULT_ALT)

for i, (lat, lon, alt) in enumerate(waypoints, start=1):
    print(f"\n===== WAYPOINT {i}/{len(waypoints)} =====")

    goto(lat, lon, alt)
    descend_and_hover(DROP_ALT, HOVER_TIME)

    print("Opening servo")
    set_servo(SERVO_OPEN, "OPEN")

    print("RTL")
    master.set_mode_apm("RTL")
    wait_until_landed()

    master.arducopter_disarm()
    print(f"Ground wait {GROUND_WAIT}s")
    time.sleep(GROUND_WAIT)

    if i < len(waypoints):
        arm_and_takeoff(alt)

# ---- MISSION TIMER END ----
mission_end_time = time.time()
elapsed = mission_end_time - mission_start_time

mins = int(elapsed // 60)
secs = int(elapsed % 60)

print("Mission complete")
print(f"Total mission time: {mins} min {secs} sec ({elapsed:.1f} s)")
