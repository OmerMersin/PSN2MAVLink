#!/usr/bin/env python3
"""
vision_only_sim.py
------------------
Send ONLY VISION_POSITION_ESTIMATE to ArduPilot for GPS-denied navigation.
This is the recommended approach for ArduSub in pools/indoor environments.

Required ArduPilot parameters:
  EK3_SRC1_POSXY = 6 (ExternalNav)
  EK3_SRC1_POSZ = 6 (ExternalNav) 
  EK3_SRC1_VELXY = 6 (ExternalNav) [optional]
  EK3_SRC1_VELZ = 0 (None)
  GPS_TYPE = 0 (None) - disable GPS entirely
  VISO_TYPE = 1 (MAV)
"""

import math
import time
import sys
import argparse
from pymavlink import mavutil


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
    parser.add_argument("--mavlink", default="udpin:0.0.0.0:14552",
                        help="MAVLink connection string (use udpin for bidirectional)")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Update rate (Hz)")
    parser.add_argument("--base-x", type=float, default=0.0, help="Base X position (m)")
    parser.add_argument("--base-y", type=float, default=0.0, help="Base Y position (m)")
    parser.add_argument("--base-z", type=float, default=-1.0, help="Base Z position (m, negative=depth)")
    args = parser.parse_args()

    print(f"Connecting to {args.mavlink} ...")
    mav = mavutil.mavlink_connection(args.mavlink)
    mav.wait_heartbeat()
    print(f"✅ Connected (system {mav.target_system}, component {mav.target_component})")

    rate = 1.0 / args.rate
    start_time = time.time()

    print(f"📡 Sending VISION_POSITION_ESTIMATE + DISTANCE_SENSOR at {args.rate:.1f} Hz... Ctrl+C to stop.")
    print(f"   Base position: X={args.base_x:.2f}m, Y={args.base_y:.2f}m, Z={args.base_z:.2f}m")
    print("\nRequired ArduPilot params:")
    print("  EK3_SRC1_POSXY = 6 (ExternalNav)")
    print("  EK3_SRC1_POSZ = 2 (RangeFinder) or 6 (ExternalNav)")
    print("  GPS_TYPE = 0 (None)")
    print("  VISO_TYPE = 1 (MAV)")
    print("  RNGFND1_TYPE = 10 (MAVLink)")
    print("  RNGFND1_MAX_CM = 1000")
    print("  RNGFND1_ORIENT = 25 (Down)\n")

    while True:
        now = time.time()
        t_rel = now - start_time
        x, y, z = generate_pose(t_rel, args.base_x, args.base_y, args.base_z)

        usec = int(now * 1e6)

        # Covariance array (21 elements = upper triangle of 6x6 matrix)
        # Set reasonable values for position uncertainty
        covariance = [0.0] * 21
        covariance[0] = 0.01   # var(x)
        covariance[2] = 0.01   # var(y) 
        covariance[5] = 0.04   # var(z)
        covariance[9] = 0.01   # var(roll)
        covariance[14] = 0.01  # var(pitch)
        covariance[20] = 0.04  # var(yaw)

        # Send VISION_POSITION_ESTIMATE with covariance
        mav.mav.vision_position_estimate_send(
            usec,    # timestamp (us)
            x,       # x (m, forward/north)
            y,       # y (m, right/east)
            z,       # z (m, down)
            0.0,     # roll (rad)
            0.0,     # pitch (rad)
            0.0,     # yaw (rad)
            covariance,  # covariance array
            0        # reset_counter
        )

        # Send fake DISTANCE_SENSOR (rangefinder pointing down)
        # Distance is positive (depth below surface)
        distance_cm = int(abs(z) * 100)  # Convert meters to centimeters
        time_boot_ms = int((now - start_time) * 1000)  # Milliseconds since start
        mav.mav.distance_sensor_send(
            time_boot_ms,      # time_boot_ms (uint32, milliseconds since boot)
            10,                # min_distance (cm)
            1000,              # max_distance (cm) 
            distance_cm,       # current_distance (cm)
            0,                 # type (0 = laser/lidar)
            0,                 # id (0 = first sensor)
            25,                # orientation (25 = MAV_SENSOR_ROTATION_PITCH_270 = downward facing)
            3,                 # covariance (cm, uncertainty)
            0.0,               # horizontal_fov (rad, 0=not applicable)
            0.0,               # vertical_fov (rad, 0=not applicable)
            [0,0,0,0],         # quaternion (not used for rangefinder)
            0                  # signal_quality (0-100%, 0=not available)
        )

        time.sleep(rate)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Stopped.")
        sys.exit(0)
