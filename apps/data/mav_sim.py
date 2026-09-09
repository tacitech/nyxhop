#!/usr/bin/env python3
"""mav_sim - a MAVLink generator that measures the link's two-way UDP pipe.

The pipe: whatever a program sends by UDP into one end comes out as the same datagram at
the other end, any content (MAVLink here only so the CRC can be checked):
  --dir rx2tx : UDP 14555 -> nyx-rx -> ground board -> control channel (small frames,
                sent twice) -> aircraft board -> nyx-tx -> UDP 14556. For commands and RC.
  --dir tx2rx : UDP 14557 -> nyx-tx -> Data frames inside the video stream (retransmitted
                like video, for larger flows) -> nyx-rx -> UDP 14558.
It emits real MAVLink v1 (X25 CRC with crc_extra): HEARTBEAT at 1 Hz plus
RC_CHANNELS_OVERRIDE at --rate Hz, reads the far end back, checks the CRC and matches the
sequence numbers, and reports loss and latency (p50/p95/max).

Use:  python mav_sim.py [--dir rx2tx|tx2rx] [--rate 10] [--duration 60]
      (nyx-tx and nyx-rx must be running; watch the 'telemetry:' lines in both apps' logs)
"""
import argparse
import socket
import struct
import time

STX_V1 = 0xFE
CRC_EXTRA = {0: 50, 70: 124, 69: 243}  # HEARTBEAT, RC_CHANNELS_OVERRIDE, MANUAL_CONTROL


def x25(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def frame(seq, sysid, compid, msgid, payload):
    hdr = bytes([len(payload), seq & 0xFF, sysid, compid, msgid])
    crc = x25(hdr + payload)
    crc = x25(bytes([CRC_EXTRA[msgid]]), crc)
    return bytes([STX_V1]) + hdr + payload + struct.pack("<H", crc)


def parse(buf):
    """Split the v1 frames in a datagram -> [(seq, msgid, payload, crc_ok)]."""
    out, i = [], 0
    while i + 8 <= len(buf):
        if buf[i] != STX_V1:
            i += 1
            continue
        n = buf[i + 1]
        end = i + 6 + n + 2
        if end > len(buf):
            break
        seq, msgid = buf[i + 2], buf[i + 5]
        payload = buf[i + 6:i + 6 + n]
        crc_rx = struct.unpack_from("<H", buf, i + 6 + n)[0]
        crc = x25(buf[i + 1:i + 6 + n])
        if msgid in CRC_EXTRA:
            crc = x25(bytes([CRC_EXTRA[msgid]]), crc)
        out.append((seq, msgid, payload, crc == crc_rx))
        i = end
    return out


def heartbeat():
    # custom_mode u32, type u8, autopilot u8, base_mode u8, system_status u8, mavlink_version u8
    return struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)  # type GCS, autopilot INVALID


def rc_override(t, sysid=1, compid=1):
    # 8 x u16 chan_raw (1000..2000), target_system u8, target_component u8
    ch = [1500 + int(400 * __import__("math").sin(t * 2.0 + k)) for k in range(8)]
    return struct.pack("<8HBB", *ch, sysid, compid)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="rx2tx", choices=["rx2tx", "tx2rx"])
    ap.add_argument("--in", dest="inp", default=None, help="UDP in (default follows --dir)")
    ap.add_argument("--out", dest="out", default=None, help="UDP out (default follows --dir)")
    ap.add_argument("--rate", type=float, default=10.0, help="RC_CHANNELS_OVERRIDE Hz")
    ap.add_argument("--duration", type=float, default=60.0)
    a = ap.parse_args()
    if a.inp is None:
        a.inp = "127.0.0.1:14555" if a.dir == "rx2tx" else "127.0.0.1:14557"
    if a.out is None:
        a.out = "0.0.0.0:14556" if a.dir == "rx2tx" else "0.0.0.0:14558"
    hi, pi = a.inp.split(":")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ho, po = a.out.split(":")
    rx.bind((ho, int(po)))
    rx.settimeout(0.02)

    sent = {}  # seq -> (t_send, msgid)
    seq = 0
    n_sent = n_rx = n_crc_bad = n_unknown = 0
    lat = []
    t0 = time.time()
    next_rc = t0
    next_hb = t0
    next_rep = t0 + 10
    win_sent = win_rx = 0
    print(f"== mav_sim {a.dir}: RC {a.rate:g} Hz + HB 1 Hz -> {a.inp}, listening on {a.out}, {a.duration:g} s ==", flush=True)
    while time.time() - t0 < a.duration:
        now = time.time()
        if now >= next_rc:
            next_rc += 1.0 / a.rate
            pkt = frame(seq, 255, 190, 70, rc_override(now))
            tx.sendto(pkt, (hi, int(pi)))
            sent[seq] = (now, 70)
            seq = (seq + 1) & 0xFF
            n_sent += 1
            win_sent += 1
        if now >= next_hb:
            next_hb += 1.0
            pkt = frame(seq, 255, 190, 0, heartbeat())
            tx.sendto(pkt, (hi, int(pi)))
            sent[seq] = (now, 0)
            seq = (seq + 1) & 0xFF
            n_sent += 1
            win_sent += 1
        try:
            d, _ = rx.recvfrom(2048)
        except socket.timeout:
            d = b""
        if d:
            for s, mid, _pl, ok in parse(d):
                n_rx += 1
                win_rx += 1
                if not ok:
                    n_crc_bad += 1
                    continue
                if s in sent and sent[s][1] == mid:
                    lat.append((time.time() - sent[s][0]) * 1000.0)
                    del sent[s]
                else:
                    n_unknown += 1
        if now >= next_rep:
            next_rep += 10
            print(f"  t={int(now - t0):3d}s sent {win_sent:3d} received {win_rx:3d}"
                  + (f"  latency p50 {sorted(lat[-win_rx:])[len(lat[-win_rx:]) // 2]:.0f} ms" if win_rx and lat else ""),
                  flush=True)
            win_sent = win_rx = 0
    time.sleep(1.5)
    while True:  # drain what is still in flight
        try:
            d, _ = rx.recvfrom(2048)
        except socket.timeout:
            break
        for s, mid, _pl, ok in parse(d):
            n_rx += 1
            if ok and s in sent and sent[s][1] == mid:
                lat.append((time.time() - sent[s][0]) * 1000.0)
                del sent[s]
    lat.sort()
    loss = n_sent - len(lat)
    print("")
    print(f"== TOTAL: sent {n_sent} | received intact {len(lat)} ({100.0 * len(lat) / max(1, n_sent):.1f}%) "
          f"| lost {loss} ({100.0 * loss / max(1, n_sent):.1f}%) | bad CRC {n_crc_bad} | unknown {n_unknown}", flush=True)
    if lat:
        print(f"   latency ms: p50 {lat[len(lat) // 2]:.0f} | p95 {lat[int(len(lat) * 0.95)]:.0f} | max {lat[-1]:.0f}", flush=True)


if __name__ == "__main__":
    main()
