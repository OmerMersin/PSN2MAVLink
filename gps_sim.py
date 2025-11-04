#!/usr/bin/env python3
"""
psn_gps_bridge.py
-----------------
Feeds ArduSub with simulated or tag-based position data.

✅ Features:
 - Connects to ArduPilot via MAVLink (UDP/serial)
 - Optionally sets EKF origin (Set EKF Origin on map)
 - Converts local PSN (x,y,z) → fake GPS via GPS_INPUT
 - Sends optional VISION_POSITION_ESTIMATE for EKF fusion
 - Unlocks GUIDED/AUTO and click-to-go navigation in Mission Planner
"""

import math
import time
import socket
import struct
import sys
import argparse
from pymavlink import mavutil

EARTH_RADIUS = 6378137.0


def local_to_latlon(lat0, lon0, x_n, y_e):
    """Convert local NED offset (m) to lat/lon."""
    dlat = x_n / EARTH_RADIUS
    dlon = y_e / (EARTH_RADIUS * math.cos(math.radians(lat0)))
    return lat0 + math.degrees(dlat), lon0 + math.degrees(dlon)


def generate_pose(t, base_x, base_y, base_z):
    """Simulate small drift/jitter for testing."""
    JITTER_AMP = 0.02
    JITTER_FREQ = 5.0
    DRIFT_AMP = 0.08
    DRIFT_FREQ = 0.15

    jit_x = JITTER_AMP * math.sin(2 * math.pi * JITTER_FREQ * t)
    jit_y = JITTER_AMP * math.cos(2 * math.pi * (JITTER_FREQ * 0.9) * t)
    drift_x = DRIFT_AMP * math.sin(2 * math.pi * DRIFT_FREQ * t)
    drift_y = DRIFT_AMP * math.cos(2 * math.pi * DRIFT_FREQ * t)

    return base_x + jit_x + drift_x, base_y + jit_y + drift_y, base_z


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", default="udp:127.0.0.1:14552",
                        help="MAVLink connection string")
    parser.add_argument("--origin-lat", type=float, default=37.4173190,
                        help="Origin latitude (deg)")
    parser.add_argument("--origin-lon", type=float, default=-6.0047261,
                        help="Origin longitude (deg)")
    parser.add_argument("--origin-alt", type=float, default=10.0,
                        help="Origin altitude (m MSL)")
    parser.add_argument("--set-ekf-origin", action="store_true",
                        help="Send SET_GPS_GLOBAL_ORIGIN to ArduPilot")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Update rate (Hz)")
    parser.add_argument("--send-vision", action="store_true",
                        help="Also send VISION_POSITION_ESTIMATE")
    args = parser.parse_args()

    print(f"Connecting to {args.mavlink} ...")
    mav = mavutil.mavlink_connection(args.mavlink)
    mav.wait_heartbeat()
    print(f"✅ Connected (system {mav.target_system}, component {mav.target_component})")

    # Optionally set EKF origin
    if args.set_ekf_origin:
        print(f"Setting EKF origin to lat={args.origin_lat:.7f}, lon={args.origin_lon:.7f}, alt={args.origin_alt:.1f}")
        mav.mav.set_gps_global_origin_send(
            mav.target_system,
            int(args.origin_lat * 1e7),
            int(args.origin_lon * 1e7),
            int(args.origin_alt * 1000)
        )
        time.sleep(0.5)
        print("✅ EKF origin set")

    # Request data streams
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)

    rate = 1.0 / args.rate
    base_x, base_y, base_z = 0.0, 0.0, 0.0
    start_time = time.time()

    print(f"📡 Sending fake GPS + vision at {args.rate:.1f} Hz... Ctrl+C to stop.")
    while True:
        now = time.time()
        t_rel = now - start_time
        x, y, z = generate_pose(t_rel, base_x, base_y, base_z)

        # Convert to global GPS
        lat, lon = local_to_latlon(args.origin_lat, args.origin_lon, x, y)
        alt = args.origin_alt - z

        usec = int(now * 1e6)

        # ---- Send GPS_INPUT ----
        mav.mav.gps_input_send(
            int(usec),           # time_usec (uint64)
            0,                   # gps_id (uint8)
            0,                   # ignore_flags (uint16)
            0,                   # time_week_ms (uint32)
            0,                   # time_week (uint16)
            3,                   # fix_type (uint8) - 3D fix
            int(lat * 1e7),      # lat (int32, degE7)
            int(lon * 1e7),      # lon (int32, degE7)
            float(alt),          # alt (float, m AMSL)
            1.0,                 # hdop (float)
            1.0,                 # vdop (float)
            0.0,                 # vn - velocity north (float, m/s)
            0.0,                 # ve - velocity east (float, m/s)
            0.0,                 # vd - velocity down (float, m/s)
            0.5,                 # speed_accuracy (float)
            0.5,                 # horiz_accuracy (float)
            0.5,                 # vert_accuracy (float)
            10,                  # satellites_visible (uint8)
            0                    # yaw (uint16, cdeg) - 0 = unknown
        )

        # ---- Optionally send VISION_POSITION_ESTIMATE ----
        if args.send_vision:
            mav.mav.vision_position_estimate_send(
                usec, x, y, z, 0.0, 0.0, 0.0
            )

        time.sleep(rate)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Stopped.")
        sys.exit(0)
