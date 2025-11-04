from pymavlink import mavutil
import time
import math

# Set your manual origin here (latitude, longitude, altitude MSL in meters)
# You can get these by right-clicking on Mission Planner map
MANUAL_ORIGIN_LAT = 0.0  # degrees
MANUAL_ORIGIN_LON = 0.0  # degrees  
MANUAL_ORIGIN_ALT = 0.0  # meters MSL

# Or set to None to use current position as origin (if you have GPS at startup)
USE_CURRENT_POSITION_AS_ORIGIN = True

print("Connecting to MAVLink at tcp:127.0.0.1:5762...")
try:
    mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
    print("Waiting for heartbeat...")
    if mav.wait_heartbeat(timeout=5) is None:
        print("ERROR: No heartbeat received. Is MAVProxy/ArduPilot running?")
        print("Start MAVProxy first: python -m MAVProxy.mavproxy --master=COM11,115200 --out=tcp:127.0.0.1:5762")
        exit(1)
    print(f"Connected (system {mav.target_system}, component {mav.target_component})")
except Exception as e:
    print(f"ERROR connecting to MAVLink: {e}")
    print("Make sure MAVProxy is running on tcp:127.0.0.1:5762")
    exit(1)

# Request attitude stream for yaw
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    10, 1
)

# If using current position, request GPS briefly
if USE_CURRENT_POSITION_AS_ORIGIN:
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        10, 1
    )

fc_yaw = 0.0
SMOOTH = 0.9
send_period = 0.05  # 20 Hz
next_send = time.time() + send_period

# Wobble params
JITTER_AMP = 0.02
JITTER_FREQ = 6.0
DRIFT_AMP = 0.08
DRIFT_FREQ = 0.15

start_time = time.time()
origin_set = False
origin_lat = MANUAL_ORIGIN_LAT
origin_lon = MANUAL_ORIGIN_LON
origin_alt = MANUAL_ORIGIN_ALT

# Vision position starts at origin (0,0,0 in local frame)
base_x = base_y = base_z = 0.0

# Get origin from GPS if requested
if USE_CURRENT_POSITION_AS_ORIGIN:
    print("Waiting for GPS position to set origin...")
    while not origin_set:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            origin_lat = msg.lat / 1e7
            origin_lon = msg.lon / 1e7
            origin_alt = msg.alt / 1000.0  # MSL altitude
            origin_set = True
            print(f"Origin set from GPS: lat={origin_lat:.7f}, lon={origin_lon:.7f}, alt={origin_alt:.1f}m")
else:
    origin_set = True
    print(f"Using manual origin: lat={origin_lat:.7f}, lon={origin_lon:.7f}, alt={origin_alt:.1f}m")

# Set EKF origin via MAVLink
print("Setting EKF origin...")
mav.mav.set_gps_global_origin_send(
    mav.target_system,
    int(origin_lat * 1e7),
    int(origin_lon * 1e7),
    int(origin_alt * 1000)  # millimeters
)
time.sleep(0.5)

print("Sending vision position updates at 20 Hz...")
print("Vision position will stay near (0,0,0) with small jitter/drift")

while True:
    now = time.time()
    
    # Get yaw updates
    msg = mav.recv_match(blocking=False)
    if msg:
        t = msg.get_type()
        if t == "ATTITUDE":
            fc_yaw = msg.yaw

    if now >= next_send:
        t_rel = now - start_time

        # Add jitter and drift to base position (staying near origin)
        jit_x = JITTER_AMP * math.sin(2.0 * math.pi * JITTER_FREQ * t_rel)
        jit_y = JITTER_AMP * math.cos(2.0 * math.pi * (JITTER_FREQ * 0.85) * t_rel)
        drift_x = DRIFT_AMP * math.sin(2.0 * math.pi * DRIFT_FREQ * t_rel)
        drift_y = DRIFT_AMP * math.cos(2.0 * math.pi * DRIFT_FREQ * t_rel)

        vis_x = base_x + jit_x + drift_x
        vis_y = base_y + jit_y + drift_y
        vis_z = base_z

        usec = int(now * 1e6)
        mav.mav.vision_position_estimate_send(
            usec,
            vis_x, vis_y, vis_z,
            0.0, 0.0, fc_yaw
        )

        next_send += send_period

    time.sleep(0.005)
