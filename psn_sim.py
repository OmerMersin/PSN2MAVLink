# psn_sim.py
# Minimal PosiStageNet (PSN) v2-ish simulator: sends tracker XYZ over UDP.
# Matches the parsing assumptions in your C++ bridge:
# - Root chunk (0x6755) data_len == bytes of the *children area only* (excludes the header chunk).
# - Immediately after root header: a PSN_DATA_PACKET_HEADER chunk (0x0000).
# - Then a TRACKER_LIST (0x0001) containing TRACKER <id> with TRACKER_POS (0x0000).

import argparse, math, socket, struct, sys, time

PSN_DATA_PACKET        = 0x6755
PSN_DATA_PACKET_HEADER = 0x0000
PSN_DATA_TRACKER_LIST  = 0x0001
PSN_DATA_TRACKER_POS   = 0x0000
PSN_DATA_TRACKER_SPEED = 0x0001  # unused here

def build_packet(ts_us, frame_id, trackers):
    """
    trackers: list of (tracker_id, (x,y,z), optional (vx,vy,vz) or None)
    Returns raw bytes of one PSN data packet compatible with your parser.
    """

    # --- Build TRACKER_LIST payload ---
    tracker_list_payload = bytearray()

    for tid, pos, vel in trackers:
        # Build subchunks for this tracker
        sub = bytearray()

        # POS subchunk: id=0x0000, len=12 (3 floats)
        pos_payload = struct.pack("<fff", *pos)
        sub += struct.pack("<HH", PSN_DATA_TRACKER_POS, len(pos_payload)) + pos_payload

        # (Optional) SPEED subchunk
        if vel is not None:
            vel_payload = struct.pack("<fff", *vel)
            sub += struct.pack("<HH", PSN_DATA_TRACKER_SPEED, len(vel_payload)) + vel_payload

        # Tracker chunk: id = tracker_id, len = len(sub)  (flag bit not required by your parser)
        tracker_list_payload += struct.pack("<HH", tid & 0xFFFF, len(sub)) + sub

    # TRACKER_LIST chunk wraps all tracker chunks
    tracker_list_chunk = struct.pack("<HH", PSN_DATA_TRACKER_LIST, len(tracker_list_payload)) + tracker_list_payload

    # --- Packet header chunk (timestamp, version, frame) ---
    # DataPacketHeader = <QBBBB  (uint64 ts_us, u8 v_hi, u8 v_lo, u8 frame_id, u8 frame_packet_count)
    dph_payload = struct.pack("<QBBBB", ts_us, 2, 2, frame_id & 0xFF, 1)
    dph_chunk   = struct.pack("<HH", PSN_DATA_PACKET_HEADER, len(dph_payload)) + dph_payload

    # --- Root header (IMPORTANT: your parser counts root.data_len from AFTER the header chunk) ---
    # So we set root.data_len == len(TRACKER_LIST chunk only), and place dph_chunk before it.
    children_region = tracker_list_chunk
    root_header     = struct.pack("<HH", PSN_DATA_PACKET, len(children_region))

    return root_header + dph_chunk + children_region

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group", default="236.10.10.10", help="multicast group (or unicast IP)")
    ap.add_argument("--port", type=int, default=56565, help="UDP port")
    ap.add_argument("--iface", default=None, help="bind interface IP for multicast (optional)")
    ap.add_argument("--tracker", type=int, default=1, help="tracker ID to publish")
    ap.add_argument("--rate", type=float, default=30.0, help="Hz")
    ap.add_argument("--radius", type=float, default=2.0, help="circle radius (m)")
    ap.add_argument("--height", type=float, default=1.5, help="Z (m, up)")
    ap.add_argument("--unicast", action="store_true", help="force unicast (don’t set multicast TTL)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Multicast nic binding (optional)
    if not args.unicast:
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
            if args.iface:
                sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(args.iface))
        except OSError:
            pass

    addr = (args.group, args.port)

    print(f"Sending PSN to {addr} (tracker {args.tracker}) at {args.rate:.1f} Hz")
    t0 = time.perf_counter()
    frame = 0
    dt = 1.0/args.rate

    while True:
        t = time.perf_counter() - t0
        # Simple circular motion in XY, constant Z (Z up)
        x = args.radius * math.cos(0.5 * t)
        y = args.radius * math.sin(0.5 * t)
        z = args.height

        # Optional velocities (derivatives of above), not required for your current bridge
        vx = -0.5 * args.radius * math.sin(0.5 * t)
        vy =  0.5 * args.radius * math.cos(0.5 * t)
        vz = 0.0

        ts_us = int((time.perf_counter()) * 1e6)  # pseudo timestamp
        pkt = build_packet(ts_us, frame, [(args.tracker, (x, y, z), None)])  # set vel to (vx,vy,vz) if you enable it

        sock.sendto(pkt, addr)
        frame = (frame + 1) & 0xFF
        time.sleep(dt)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
