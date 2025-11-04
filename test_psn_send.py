#!/usr/bin/env python3
"""Simple PSN packet sender for testing"""
import socket
import struct
import time

def send_test_packet(group, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    
    # Build minimal PSN packet
    PSN_DATA_PACKET = 0x6755
    PSN_DATA_PACKET_HEADER = 0x0000
    PSN_DATA_TRACKER_LIST = 0x0001
    PSN_DATA_TRACKER_POS = 0x0000
    
    # Tracker position
    tracker_id = 1
    x, y, z = 1.0, 2.0, 3.0
    tracker_pos_payload = struct.pack("<fff", x, y, z)
    tracker_pos_chunk = struct.pack("<HH", PSN_DATA_TRACKER_POS, len(tracker_pos_payload)) + tracker_pos_payload
    
    # Tracker chunk
    tracker_chunk = struct.pack("<HH", tracker_id, len(tracker_pos_chunk)) + tracker_pos_chunk
    
    # Tracker list
    tracker_list_chunk = struct.pack("<HH", PSN_DATA_TRACKER_LIST, len(tracker_chunk)) + tracker_chunk
    
    # Data packet header
    timestamp_us = int(time.time() * 1e6)
    dph_payload = struct.pack("<QBBBB", timestamp_us, 2, 2, 0, 1)
    dph_chunk = struct.pack("<HH", PSN_DATA_PACKET_HEADER, len(dph_payload)) + dph_payload
    
    # Root packet
    children = dph_chunk + tracker_list_chunk
    root = struct.pack("<HH", PSN_DATA_PACKET, len(children)) + children
    
    print(f"Sending {len(root)} bytes to {group}:{port}")
    print(f"Tracker 1: pos=({x}, {y}, {z})")
    
    for i in range(5):
        sock.sendto(root, (group, port))
        print(f"Sent packet {i+1}")
        time.sleep(0.5)
    
    sock.close()

if __name__ == "__main__":
    send_test_packet("236.10.10.10", 56565)