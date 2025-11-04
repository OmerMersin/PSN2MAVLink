# numb_psn_sim.py
# Emit pseudo PSN packets that mimic the vision pose jitter/drift from numb_psn.

import argparse
import math
import socket
import struct
import sys
import time

from typing import Any

from pymavlink import mavutil

PSN_DATA_PACKET = 0x6755
PSN_DATA_PACKET_HEADER = 0x0000
PSN_DATA_TRACKER_LIST = 0x0001
PSN_DATA_TRACKER_POS = 0x0000

SMOOTH = 0.9
JITTER_AMP = 0.005  # Reduced for GPS-denied mode (was 0.02)
JITTER_FREQ = 6.0
DRIFT_AMP = 0.02    # Reduced for GPS-denied mode (was 0.08)
DRIFT_FREQ = 0.15
EARTH_RADIUS = 6378137.0
DEFAULT_MAVLINK_URL = "udp:127.0.0.1:14552"
DEFAULT_RATE = 20.0


def build_packet(ts_us: int, frame_id: int, tracker_id: int, pos):
    """Format one PSN packet with a single tracker position."""
    tracker_payload = struct.pack("<fff", *pos)
    tracker_chunk = struct.pack("<HH", PSN_DATA_TRACKER_POS, len(tracker_payload)) + tracker_payload
    tracker_list = struct.pack("<HH", tracker_id & 0xFFFF, len(tracker_chunk)) + tracker_chunk
    tracker_list_chunk = struct.pack("<HH", PSN_DATA_TRACKER_LIST, len(tracker_list)) + tracker_list

    dph_payload = struct.pack("<QBBBB", ts_us, 2, 2, frame_id & 0xFF, 1)
    dph_chunk = struct.pack("<HH", PSN_DATA_PACKET_HEADER, len(dph_payload)) + dph_payload

    root_header = struct.pack("<HH", PSN_DATA_PACKET, len(tracker_list_chunk))
    return root_header + dph_chunk + tracker_list_chunk



def latlon_to_local_m(lat0: float, lon0: float, lat: float, lon: float):
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = dlat * EARTH_RADIUS
    y = dlon * EARTH_RADIUS * math.cos(math.radians(lat0))
    return x, y


def generate_pose(t_rel: float, base_x: float, base_y: float, base_z: float):
    """Replica of numb_psn: add jitter and slow drift around the filtered FC pose."""
    jit_x = JITTER_AMP * math.sin(2.0 * math.pi * JITTER_FREQ * t_rel)
    jit_y = JITTER_AMP * math.cos(2.0 * math.pi * (JITTER_FREQ * 0.85) * t_rel)

    drift_x = DRIFT_AMP * math.sin(2.0 * math.pi * DRIFT_FREQ * t_rel)
    drift_y = DRIFT_AMP * math.cos(2.0 * math.pi * DRIFT_FREQ * t_rel)

    vis_x = base_x + jit_x + drift_x
    vis_y = base_y + jit_y + drift_y
    vis_z = base_z
    return vis_x, vis_y, vis_z


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--group", default="236.10.10.10", help="multicast group (or unicast IP)")
    parser.add_argument("--port", type=int, default=56565, help="UDP port")
    parser.add_argument("--iface", default=None, help="bind interface IP for multicast")
    parser.add_argument("--tracker", type=int, default=1, help="tracker ID to publish")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE, help="update rate (Hz)")
    parser.add_argument("--mavlink", default=DEFAULT_MAVLINK_URL, help="MAVLink connection string")
    parser.add_argument("--no-stream-req", action="store_true", help="skip requesting data streams")
    parser.add_argument("--no-mavlink", action="store_true", help="skip MAVLink connection, use static base position")
    parser.add_argument("--base-x", type=float, default=0.0, help="base X position (m, used with --no-mavlink)")
    parser.add_argument("--base-y", type=float, default=0.0, help="base Y position (m, used with --no-mavlink)")
    parser.add_argument("--base-z", type=float, default=-1.0, help="base Z position (m, used with --no-mavlink)")
    parser.add_argument("--set-ekf-origin", action="store_true", help="set EKF origin via MAVLink (for GPS-denied ops)")
    parser.add_argument("--origin-lat", type=float, default=0.0, help="EKF origin latitude (degrees)")
    parser.add_argument("--origin-lon", type=float, default=0.0, help="EKF origin longitude (degrees)")
    parser.add_argument("--origin-alt", type=float, default=0.0, help="EKF origin altitude MSL (meters)")
    parser.add_argument("--unicast", action="store_true", help="force unicast mode")
    args = parser.parse_args()

    mav: Any = None
    if not args.no_mavlink:
        print(f"Connecting to MAVLink at {args.mavlink}...")
        try:
            mav = mavutil.mavlink_connection(args.mavlink)
            print("Waiting for heartbeat...")
            if mav.wait_heartbeat(timeout=5) is None:  # type: ignore[attr-defined]
                print("ERROR: No heartbeat received. Is MAVProxy/ArduPilot running?")
                print("Start MAVProxy first: python -m MAVProxy.mavproxy --master=COM11,115200 --out=tcp:127.0.0.1:5762")
                sys.exit(1)
            print(f"Connected to MAVLink (system {mav.target_system}, component {mav.target_component})")  # type: ignore[attr-defined]
        except Exception as e:
            print(f"ERROR connecting to MAVLink: {e}")
            print("Make sure MAVProxy is running on tcp:127.0.0.1:5762")
            sys.exit(1)
        
        # Set EKF origin if requested (for GPS-denied navigation)
        if args.set_ekf_origin:
            print(f"Setting EKF origin: lat={args.origin_lat:.7f}, lon={args.origin_lon:.7f}, alt={args.origin_alt:.1f}m")
            mav.mav.set_gps_global_origin_send(  # type: ignore[attr-defined]
                mav.target_system,  # type: ignore[attr-defined]
                int(args.origin_lat * 1e7),
                int(args.origin_lon * 1e7),
                int(args.origin_alt * 1000)
            )
            time.sleep(0.5)
            print("EKF origin set successfully")

    if mav and not args.no_stream_req:
        mav.mav.request_data_stream_send(  # type: ignore[attr-defined]
            mav.target_system,  # type: ignore[attr-defined]
            mav.target_component,  # type: ignore[attr-defined]
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1,
        )
        mav.mav.request_data_stream_send(  # type: ignore[attr-defined]
            mav.target_system,  # type: ignore[attr-defined]
            mav.target_component,  # type: ignore[attr-defined]
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            10,
            1,
        )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if not args.unicast:
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
            if args.iface:
                sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(args.iface))
        except OSError:
            pass

    addr = (args.group, args.port)
    print(f"Sending numb_psn-like PSN data to {addr} at {args.rate:.1f} Hz (tracker {args.tracker})")

    send_period = 1.0 / args.rate
    frame = 0
    start_time = time.time()
    origin = None
    base_x = args.base_x if args.no_mavlink else 0.0
    base_y = args.base_y if args.no_mavlink else 0.0
    base_z = args.base_z if args.no_mavlink else 0.0
    next_send = time.time() + send_period

    # If not using MAVLink, mark origin as "ready" immediately
    if args.no_mavlink:
        origin = (0.0, 0.0)
        print(f"Static mode: base position ({base_x:.2f}, {base_y:.2f}, {base_z:.2f})")

    while True:
        now = time.time()

        # Process MAVLink messages if connected
        if mav:
            msg = mav.recv_match(blocking=False)
            if msg:
                mtype = msg.get_type()
                if mtype == "GLOBAL_POSITION_INT":
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

        # Send PSN packet at scheduled time
        if origin is not None and now >= next_send:
            t_rel = now - start_time
            pose = generate_pose(t_rel, base_x, base_y, base_z)
            psn_pose = (pose[0], pose[1], -pose[2])
            ts_us = int(now * 1e6)
            pkt = build_packet(ts_us, frame, args.tracker, psn_pose)
            sock.sendto(pkt, addr)
            frame = (frame + 1) & 0xFF
            next_send += send_period

        # Sleep only the remaining time until next send
        sleep_time = next_send - time.time() if origin is not None else 0.001
        if sleep_time > 0:
            time.sleep(min(sleep_time, 0.001))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
