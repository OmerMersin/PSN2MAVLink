#!/usr/bin/env python3
"""
Circle Simulator - PSN Protocol
Sends PSN tracker data with circular trajectory.
"""
import argparse
import math
import socket
import struct
import time

PSN_DATA_PACKET = 0x6755
PSN_DATA_PACKET_HEADER = 0x0000
PSN_DATA_TRACKER_LIST = 0x0001
PSN_DATA_TRACKER_POS = 0x0000


def build_packet(ts_us: int, frame_id: int, tracker_id: int, pos):
    """Format one PSN packet with a single tracker position."""
    x, y, z = pos
    
    # Tracker position payload
    tracker_pos_payload = struct.pack("<fff", x, y, z)
    tracker_pos_chunk = struct.pack("<HH", PSN_DATA_TRACKER_POS, len(tracker_pos_payload)) + tracker_pos_payload
    
    # Tracker chunk
    tracker_chunk = struct.pack("<HH", tracker_id & 0xFFFF, len(tracker_pos_chunk)) + tracker_pos_chunk
    
    # Tracker list
    tracker_list_chunk = struct.pack("<HH", PSN_DATA_TRACKER_LIST, len(tracker_chunk)) + tracker_chunk
    
    # Data packet header
    dph_payload = struct.pack("<QBBBB", ts_us, 2, 2, 0, frame_id & 0xFF)
    dph_chunk = struct.pack("<HH", PSN_DATA_PACKET_HEADER, len(dph_payload)) + dph_payload
    
    # Root packet
    children = dph_chunk + tracker_list_chunk
    root = struct.pack("<HH", PSN_DATA_PACKET, len(children)) + children
    
    return root


def main():
    parser = argparse.ArgumentParser(description='PSN Circular Motion Simulator')
    parser.add_argument('--group', default='236.10.10.10', help='Multicast group or unicast IP')
    parser.add_argument('--port', type=int, default=56565, help='UDP port')
    parser.add_argument('--tracker', type=int, default=1, help='Tracker ID')
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate in Hz')
    parser.add_argument('--radius', type=float, default=2.0, help='Circle radius in meters')
    parser.add_argument('--period', type=float, default=10.0, help='Time for one full circle in seconds')
    parser.add_argument('--center-x', type=float, default=0.0, help='Circle center X')
    parser.add_argument('--center-y', type=float, default=0.0, help='Circle center Y')
    parser.add_argument('--altitude', type=float, default=-1.0, help='Altitude (negative Z = up)')
    parser.add_argument('--unicast', action='store_true', help='Use unicast instead of multicast')
    
    args = parser.parse_args()
    
    # Setup socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    
    send_period = 1.0 / args.rate
    frame_id = 0
    start_time = time.time()
    next_send = start_time
    
    mode = "unicast" if args.unicast else "multicast"
    print(f"Sending PSN circular motion to {mode} {args.group}:{args.port}")
    print(f"Circle: radius={args.radius}m, period={args.period}s, center=({args.center_x}, {args.center_y}), alt={args.altitude}m")
    print(f"Rate: {args.rate} Hz, Tracker ID: {args.tracker}")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            now = time.time()
            if now >= next_send:
                t_rel = now - start_time
                
                # Circular motion
                angle = (2.0 * math.pi * t_rel) / args.period
                x = args.center_x + args.radius * math.cos(angle)
                y = args.center_y + args.radius * math.sin(angle)
                z = args.altitude
                
                # Build and send packet
                ts_us = int(now * 1e6)
                packet = build_packet(ts_us, frame_id, args.tracker, (x, y, z))
                sock.sendto(packet, (args.group, args.port))
                
                # Print status every second
                if frame_id % int(args.rate) == 0:
                    angle_deg = math.degrees(angle) % 360
                    print(f"[{t_rel:.1f}s] Frame {frame_id}: pos=({x:.3f}, {y:.3f}, {z:.3f}), angle={angle_deg:.1f}°")
                
                frame_id += 1
                next_send += send_period
            
            sleep_time = max(0.001, next_send - time.time())
            time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        sock.close()


if __name__ == '__main__':
    main()