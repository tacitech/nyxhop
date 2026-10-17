#!/usr/bin/env python3
"""nyx_ip - an IP network over the NyxHop link (two ADRV9364 boards).

Each board becomes the router of a small network of its own: the aircraft side 192.168.10.0/24
(gateway 192.168.10.1, the aircraft board) and the ground side 192.168.20.0/24 (gateway
192.168.20.1, the ground board). The two networks meet only over the radio, so whatever runs
between a computer on one side and a computer on the other (ping, ssh, web, iperf3, file copies,
an IP camera) goes over the link, next to the video.

    set BOARD_PW=analog        # the boards' ssh password (PowerShell: $env:BOARD_PW = "analog")
    python link/scripts/nyx_ip.py on  --air 192.168.0.10 --ground 192.168.0.11
    python link/scripts/nyx_ip.py off --air 192.168.0.10 --ground 192.168.0.11

Then give each computer an address on its side: 192.168.10.50 (gateway 192.168.10.1) on the
aircraft side, 192.168.20.50 (gateway 192.168.20.1) on the ground side, and use them as any two
computers on a network. A computer that keeps its own address only needs a route, e.g. on the
ground side (Windows, as administrator): route add 192.168.10.0 mask 255.255.255.0 192.168.20.1

`on` copies the bridge program from deploy/bridge/ onto both boards and starts it; `off` stops it
and takes the addresses away. A reboot of a board turns it off as well.
"""
import argparse
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
BRIDGE_LOCAL = os.path.join(ROOT, "deploy", "bridge", "nyx-bridge")
BRIDGE = "/home/root/nyx-bridge"
LOG = "/tmp/nyx-bridge.log"
# side: (the board's end of the tunnel, the other end, this side's network, the other side's)
SIDES = {
    "air": ("10.7.0.1", "10.7.0.2", "192.168.10.1/24", "192.168.20.0/24"),
    "ground": ("10.7.0.2", "10.7.0.1", "192.168.20.1/24", "192.168.10.0/24"),
}


def connect(host):
    import warnings
    warnings.filterwarnings("ignore")
    try:
        import paramiko
    except ImportError:
        sys.exit("need paramiko:  python -m pip install paramiko")
    pw = os.environ.get("BOARD_PW")
    if not pw:
        sys.exit("set BOARD_PW to the board's ssh password first")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(host, username="root", password=pw, look_for_keys=False, allow_agent=False, timeout=10)
    return c


def sh(c, cmd, timeout=60):
    _, o, e = c.exec_command(cmd, timeout=timeout)
    return (o.read().decode(errors="replace") + e.read().decode(errors="replace")).strip()


def upload(c, data, remote):
    _, o, e = c.exec_command(f"cat > '{remote}.new'", timeout=120)
    o.channel.sendall(data)
    o.channel.shutdown_write()
    if o.channel.recv_exit_status():
        sys.exit(f"upload {remote}: {e.read().decode(errors='replace')[:200]}")
    got = sh(c, f"md5sum '{remote}.new'").split()[0]
    if got != hashlib.md5(data).hexdigest():
        sys.exit(f"upload {remote}: checksum mismatch")
    sh(c, f"chmod 755 '{remote}.new' && mv '{remote}.new' '{remote}'")


def on(host, side, data):
    me, peer, gw, far = SIDES[side]
    c = connect(host)
    try:
        upload(c, data, BRIDGE)
        extra = " --mcs auto" if side == "air" else ""
        sh(c, f"pkill -x nyx-bridge; sleep 0.5; sysctl -qw net.ipv4.ip_forward=1; "
              f"ip addr replace {gw} dev eth0; "
              f"(nohup {BRIDGE} --side {side} --board 127.0.0.1 --addr {me}/24 --peer {peer}{extra} > {LOG} 2>&1 &); "
              f"sleep 2; ip route replace {far} dev nyx0; ip link set nyx0 txqueuelen 300")
        up = sh(c, "ip link show nyx0 >/dev/null 2>&1 && echo up || echo down")
        if up != "up":
            sys.exit(f"{host}: the bridge did not start, see {LOG} on the board")
    finally:
        c.close()


def off(host, side):
    gw = SIDES[side][2]
    try:
        c = connect(host)
    except Exception as e:  # noqa: BLE001 - a board that is off has nothing to turn off
        print(f"{host}: {e}")
        return
    sh(c, f"pkill -x nyx-bridge; ip addr del {gw} dev eth0 2>/dev/null; sysctl -qw net.ipv4.ip_forward=0; true")
    c.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["on", "off"])
    ap.add_argument("--air", required=True, help="the aircraft (transmitting) board's address")
    ap.add_argument("--ground", required=True, help="the ground (receiving) board's address")
    a = ap.parse_args()
    if a.action == "off":
        off(a.air, "air")
        off(a.ground, "ground")
        print("IP bridge off.")
        return
    if not os.path.exists(BRIDGE_LOCAL):
        sys.exit(f"missing {BRIDGE_LOCAL}: run this from a checkout of the repository")
    data = open(BRIDGE_LOCAL, "rb").read()
    on(a.air, "air", data)
    on(a.ground, "ground", data)
    c = connect(a.ground)
    try:
        sh(c, "nc -w 2 127.0.0.1 7202 </dev/null >/dev/null 2>&1; true")
        _, o, _ = c.exec_command("echo 'uplink mcs auto' | nc -w 2 127.0.0.1 7202", timeout=10)
        o.read()
        p = sh(c, "ping -c 4 -i 0.3 -W 2 -I 192.168.20.1 192.168.10.1 | tail -1")
    finally:
        c.close()
    print("IP bridge on.")
    print("  aircraft side: 192.168.10.0/24, gateway 192.168.10.1")
    print("  ground side:   192.168.20.0/24, gateway 192.168.20.1")
    print(f"  ground -> aircraft over the link: {p or 'no answer yet (check that the boards are linked)'}")


if __name__ == "__main__":
    main()
