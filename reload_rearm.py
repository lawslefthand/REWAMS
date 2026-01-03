from pymavlink import mavutil
import time
import math
import csv

# ===============================
# CONFIG
# ===============================
CSV_PATH = "/home/danba/reload_rearm.csv"
DEFAULT_ALT = 20          # meters
GROUND_WAIT = 40          # seconds
ARRIVAL_THRESH = 2.0      # meters

SERVO_NUM = 9             # AUX1 = SERVO9
SERVO_OPEN = 2000
SERVO_CLOSE = 1000

DROP_ALT = 5.0            # meters
HOVER_TIME = 5            # seconds

# ===============================
# CSV LOADER
# ===============================
def load_csv_waypoints(path, default_alt=20):
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
    print("Closing servo 15 seconds before takeoff")
    set_servo(SERVO_CLOSE, "CLOSE (pre-takeoff)")
    time.sleep(15)

    master.set_mode_apm("GUIDED")
    time.sleep(1)

    master.arducopter_arm()
    time.sleep(3)

    print("Taking off...")
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
    print("Waiting for landing...")
    while True:
        msg = master.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True
        )
        alt = msg.relative_alt / 1000.0
        if alt < 0.15:
            print("Landed.")
            break
        time.sleep(0.5)


def descend_and_hover(target_alt, hover_time):
    print(f"Descending to {target_alt} m")

    # Get current position
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    cur_lat = msg.lat
    cur_lon = msg.lon

    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        cur_lat,
        cur_lon,
        target_alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    # Wait until altitude reached
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        alt = msg.relative_alt / 1000.0
        if abs(alt - target_alt) < 0.3:
            print(f"Reached {target_alt} m")
            break
        time.sleep(0.2)

    print(f"Hovering for {hover_time} seconds")
    time.sleep(hover_time)

    while True:
        msg = master.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True
        )
        alt = msg.relative_alt / 1000.0
        if abs(alt - target_alt) < 0.3:
            print(f"Reached {target_alt} m")
            break
        time.sleep(0.2)

    print(f"Hovering for {hover_time} seconds")
    time.sleep(hover_time)


def goto(lat, lon, alt):
    print(f"Going to {lat}, {lon}, {alt}")
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    while True:
        msg = master.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True
        )
        clat = msg.lat / 1e7
        clon = msg.lon / 1e7
        d = distance_m(clat, clon, lat, lon)

        print(f"  Distance: {d:.2f} m", end="\r")
        if d < ARRIVAL_THRESH:
            print("\nReached waypoint")

            descend_and_hover(DROP_ALT, HOVER_TIME)

            print("Opening servo after hover")
            set_servo(SERVO_OPEN, "OPEN (after hover)")
            break

        time.sleep(0.2)

# ===============================
# MAIN LOGIC
# ===============================
waypoints = load_csv_waypoints(CSV_PATH, DEFAULT_ALT)

if len(waypoints) == 0:
    raise RuntimeError("CSV has no waypoints")

print(f"Loaded {len(waypoints)} CSV waypoints")

# Initial takeoff
arm_and_takeoff(DEFAULT_ALT)

for i, (lat, lon, alt) in enumerate(waypoints, start=1):
    print(f"\n===== WAYPOINT {i}/{len(waypoints)} =====")

    goto(lat, lon, alt)

    print("RTL")
    master.set_mode_apm("RTL")

    wait_until_landed()

    print("Opening servo after landing")
    set_servo(SERVO_OPEN, "OPEN (post-landing)")

    master.arducopter_disarm()
    print(f"Waiting on ground for {GROUND_WAIT} seconds")
    time.sleep(GROUND_WAIT)

    if i < len(waypoints):
        arm_and_takeoff(alt)

print("\nMission complete. Final RTL done.")

