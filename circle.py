#!/usr/bin/env python3
"""
Circle Simulator - Direct MAVLink
Sends VISION_POSITION_ESTIMATE messages with circular trajectory directly via MAVLink.
The drone will appear to circle around a center point.
"""

from pymavlink import mavutil
import time
import math
import argparse


def main():
    parser = argparse.ArgumentParser(description='MAVLink Circular Motion Simulator')
    parser.add_argument('--mavlink', default='udpin:0.0.0.0:14552', 
                        help='MAVLink connection string (default: udpin:0.0.0.0:14552)')
    parser.add_argument('--target-system', type=int, default=1, help='Target system ID (default: 1)')
    parser.add_argument('--target-component', type=int, default=1, help='Target component ID (default: 1)')
    
    # Circle parameters
    parser.add_argument('--radius', type=float, default=2.0, help='Circle radius in meters (default: 2.0)')
    parser.add_argument('--period', type=float, default=10.0, help='Time for one full circle in seconds (default: 10)')
    parser.add_argument('--center-x', type=float, default=0.0, help='Circle center X in meters (default: 0)')
    parser.add_argument('--center-y', type=float, default=0.0, help='Circle center Y in meters (default: 0)')
    parser.add_argument('--altitude', type=float, default=-1.0, help='Altitude Z in meters, negative = down (default: -1.0)')
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate in Hz (default: 20)')
    
    args = parser.parse_args()
    
    # Connect to MAVLink
    print(f"Connecting to MAVLink at {args.mavlink}...")
    mav = mavutil.mavlink_connection(args.mavlink)
    print("Waiting for heartbeat...")
    mav.wait_heartbeat()
    print(f"Connected: system {mav.target_system}, component {mav.target_component}")
    
    print(f"Circle: radius={args.radius}m, period={args.period}s, center=({args.center_x}, {args.center_y}), alt={args.altitude}m")
    print(f"Rate: {args.rate} Hz")
    print("Sending VISION_POSITION_ESTIMATE messages...")
    print("Press Ctrl+C to stop")
    
    send_period = 1.0 / args.rate
    next_send = time.time()
    start_time = time.time()
    frame_count = 0
    
    # Calculate angular velocity (radians per second)
    angular_velocity = 2.0 * math.pi / args.period
    
    try:
        while True:
            now = time.time()
            
            if now >= next_send:
                t_rel = now - start_time
                
                # Calculate position on circle
                angle = angular_velocity * t_rel
                x = args.center_x + args.radius * math.cos(angle)
                y = args.center_y + args.radius * math.sin(angle)
                z = args.altitude
                
                # Calculate yaw (tangent to circle)
                yaw = angle + math.pi / 2  # Perpendicular to radius
                
                # Send VISION_POSITION_ESTIMATE
                timestamp_us = int(now * 1e6)
                mav.mav.vision_position_estimate_send(
                    timestamp_us,  # usec: Timestamp (microseconds, synced to UNIX time or since system boot)
                    x,  # x: Global X position
                    y,  # y: Global Y position  
                    z,  # z: Global Z position
                    0.0,  # roll: Roll angle in rad
                    0.0,  # pitch: Pitch angle in rad
                    yaw   # yaw: Yaw angle in rad
                )
                
                # Print status every second
                if frame_count % int(args.rate) == 0:
                    print(f"[{t_rel:.1f}s] Frame {frame_count}: pos=({x:.3f}, {y:.3f}, {z:.3f}), angle={math.degrees(angle):.1f}°, yaw={math.degrees(yaw):.1f}°")
                
                frame_count += 1
                next_send += send_period
            
            # Sleep for a short time to avoid busy-waiting
            sleep_time = next_send - time.time()
            if sleep_time > 0:
                time.sleep(min(sleep_time, 0.001))
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        mav.close()


if __name__ == '__main__':
    main()
