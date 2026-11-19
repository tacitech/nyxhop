#!/usr/bin/env python3
"""DHCP for the one computer on the board's USB network: it always gets the same address, with no
gateway and no DNS, so the computer keeps its own way to the internet.

    nyx-usbdhcp.py <interface> <the board's address> <the computer's address>
"""
import socket
import struct
import sys

IFACE, ME, PEER = sys.argv[1], socket.inet_aton(sys.argv[2]), socket.inet_aton(sys.argv[3])
MASK, LEASE = socket.inet_aton("255.255.255.0"), 3600
MAGIC = b"\x63\x82\x53\x63"
DISCOVER, OFFER, REQUEST, ACK, NAK = 1, 2, 3, 5, 6

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, IFACE.encode() + b"\0")
s.bind(("", 67))
while True:
    data, _ = s.recvfrom(1500)
    if len(data) < 240 or data[0] != 1 or data[236:240] != MAGIC:
        continue
    opts, i = {}, 240
    while i + 1 < len(data) and data[i] != 255:
        if data[i] == 0:
            i += 1
            continue
        opts[data[i]] = data[i + 2:i + 2 + data[i + 1]]
        i += 2 + data[i + 1]
    kind = opts.get(53, b"\0")[0]
    if kind == DISCOVER:
        reply = OFFER
    elif kind == REQUEST:
        asked = opts.get(50, data[12:16])                 # requested address, else the one it has (renewing)
        reply = ACK if asked in (PEER, b"\0\0\0\0") else NAK
    else:
        continue
    yours = b"\0\0\0\0" if reply == NAK else PEER
    head = struct.pack("!BBBB4sHH4s4s4s4s16s192s4s", 2, data[1], data[2], 0, data[4:8], 0, 0x8000,
                       b"\0\0\0\0", yours, ME, b"\0\0\0\0", data[28:44], b"", MAGIC)
    o = bytes([53, 1, reply, 54, 4]) + ME
    if reply != NAK:
        o += bytes([51, 4]) + struct.pack("!I", LEASE) + bytes([1, 4]) + MASK
    pkt = head + o + b"\xff"
    s.sendto(pkt.ljust(300, b"\0"), ("255.255.255.255", 68))
