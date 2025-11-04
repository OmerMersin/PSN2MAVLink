from pymavlink import mavutil
import time, math

# listen for MAVProxy on 14600
mav = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
mav.wait_heartbeat()
print("Connected:", mav.target_system, mav.target_component)


# request data
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    10, 1
)
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    10, 1
)

EARTH_RADIUS = 6378137.0

def latlon_to_local_m(lat0, lon0, lat, lon):
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = dlat * EARTH_RADIUS
    y = dlon * EARTH_RADIUS * math.cos(math.radians(lat0))
    return x, y

origin = None
base_x = base_y = base_z = 0.0
fc_yaw = 0.0

SMOOTH = 0.9        # follow FC very closely
send_period = 0.05  # 20 Hz
next_send = time.time() + send_period

# ---- wobble params ----
# tiny fast jitter (sensor noise)
JITTER_AMP = 0.02        # 2 cm
JITTER_FREQ = 6.0        # Hz

# slow PSN-like breathing
DRIFT_AMP = 0.08         # 8 cm
DRIFT_FREQ = 0.15        # Hz (~6.6 s per cycle)

start_time = time.time()

while True:
    now = time.time()
    msg = mav.recv_match(blocking=False)
    if msg:
        t = msg.get_type()
        if t == "ATTITUDE":
            fc_yaw = msg.yaw
        elif t == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            if origin is None:
                origin = (lat, lon)
                base_x = 0.0
                base_y = 0.0
                base_z = -alt
                print("Origin set:", origin)
            else:
                x_m, y_m = latlon_to_local_m(origin[0], origin[1], lat, lon)
                z_m = -alt
                base_x += (x_m - base_x) * SMOOTH
                base_y += (y_m - base_y) * SMOOTH
                base_z += (z_m - base_z) * SMOOTH

    if origin is not None and now >= next_send:
        t_rel = now - start_time

        # ---- 1) fast tiny jitter (sin/cos, different freq) ----
        jit_x = JITTER_AMP * math.sin(2.0 * math.pi * JITTER_FREQ * t_rel)
        jit_y = JITTER_AMP * math.cos(2.0 * math.pi * (JITTER_FREQ * 0.85) * t_rel)

        # ---- 2) slow drift circle ----
        drift_x = DRIFT_AMP * math.sin(2.0 * math.pi * DRIFT_FREQ * t_rel)
        drift_y = DRIFT_AMP * math.cos(2.0 * math.pi * DRIFT_FREQ * t_rel)

        vis_x = base_x + jit_x + drift_x
        vis_y = base_y + jit_y + drift_y
        vis_z = base_z  # you can add small z wobble too if you want

        usec = int(now * 1e6)
        mav.mav.vision_position_estimate_send(
            usec,
            vis_x, vis_y, vis_z,
            0.0, 0.0, fc_yaw
        )

        # print(f"VISION x={vis_x:.3f} y={vis_y:.3f} z={vis_z:.3f}")
        next_send += send_period

    time.sleep(0.005)
