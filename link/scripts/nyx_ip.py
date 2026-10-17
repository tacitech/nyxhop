#!/usr/bin/env python3
"""nyx_ip - an IP network over the NyxHop link (two ADRV9364 boards).

Each board becomes the router of a small network of its own: the aircraft side 192.168.10.0/24
(gateway 192.168.10.1, the aircraft board) and the ground side 192.168.20.0/24 (gateway
192.168.20.1, the ground board). The two networks meet only over the radio, so whatever runs
between a computer on one side and a computer on the other (ping, ssh, web, iperf3, file copies,
an IP camera) goes over the link, next to the video.

    set BOARD_PW=analog        # the boards' ssh password (PowerShell: $env:BOARD_PW = "analog")
    python link/scripts/nyx_ip.py on   --air 192.168.0.10 --ground 192.168.0.11
    python link/scripts/nyx_ip.py test --air 192.168.0.10 --ground 192.168.0.11
    python link/scripts/nyx_ip.py off  --air 192.168.0.10 --ground 192.168.0.11

Then give each computer an address on its side: 192.168.10.50 (gateway 192.168.10.1) on the
aircraft side, 192.168.20.50 (gateway 192.168.20.1) on the ground side, and use them as any two
computers on a network. A computer that keeps its own address only needs a route, e.g. on the
ground side (Windows, as administrator): route add 192.168.10.0 mask 255.255.255.0 192.168.20.1

`on` copies the bridge program from deploy/bridge/ onto both boards and starts it; `test` sends a
ping and a TCP stream each way between the two networks, over the radio, and prints what got
through; `off` stops it and takes the addresses away. A reboot of a board turns it off as well.
"""
import argparse
import hashlib
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
BRIDGE_LOCAL = os.path.join(ROOT, "deploy", "bridge", "nyx-bridge")
BRIDGE = "/home/root/nyx-bridge"
LOG = "/tmp/nyx-bridge.log"
TPUT = "/tmp/nyx-tput.py"
# a plain TCP stream for `test` (the boards have python3, not iperf3):
#   rx <bind ip> <port> <secs>          prints Mbit/s each second and the total
#   tx <src ip> <dst ip> <port> <secs>
TPUT_SRC = b"""import socket, sys, time
if sys.argv[1] == "rx":
    ip, port, secs = sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
    ls = socket.socket()
    ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ls.bind((ip, port))
    ls.listen(1)
    ls.settimeout(secs + 20)
    c, _ = ls.accept()
    c.settimeout(5)
    t0 = tick = time.time()
    n = tot = 0
    while time.time() - t0 < secs:
        try:
            d = c.recv(1 << 16)
        except socket.timeout:
            break
        if not d:
            break
        n += len(d)
        tot += len(d)
        if time.time() - tick >= 1.0:
            print("  %.1f Mbit/s" % (n * 8 / (time.time() - tick) / 1e6), flush=True)
            tick, n = time.time(), 0
    dt = time.time() - t0
    print("  total %.1f Mbit/s over %.0f s" % (tot * 8 / dt / 1e6, dt), flush=True)
else:
    src, dst, port, secs = sys.argv[2], sys.argv[3], int(sys.argv[4]), float(sys.argv[5])
    s = socket.socket()
    s.bind((src, 0))
    s.connect((dst, port))
    buf = b"U" * (1 << 16)
    t0 = time.time()
    while time.time() - t0 < secs:
        s.sendall(buf)
    s.close()
"""
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


def tcp(rx_host, rx_ip, tx_host, tx_ip, port, secs):
    """a TCP stream from tx_ip (on tx_host) to rx_ip (on rx_host); the receiver prints the rate"""
    def receive():
        c = connect(rx_host)
        _, o, _ = c.exec_command(f"python3 -u {TPUT} rx {rx_ip} {port} {secs}", timeout=secs + 40)
        for line in iter(o.readline, ""):
            print(line.rstrip(), flush=True)
        c.close()
    t = threading.Thread(target=receive)
    t.start()
    time.sleep(1.5)
    c = connect(tx_host)
    c.exec_command(f"timeout {secs + 5} python3 {TPUT} tx {tx_ip} {rx_ip} {port} {secs}", timeout=secs + 20)[1].read()
    c.close()
    t.join()


def test(air, ground, secs):
    for h in (air, ground):
        c = connect(h)
        try:
            if sh(c, "ip link show nyx0 >/dev/null 2>&1 && echo up || echo down") != "up":
                sys.exit(f"{h}: the bridge is not on, run `nyx_ip.py on` first")
            upload(c, TPUT_SRC, TPUT)
        finally:
            c.close()
    air_ip, gnd_ip = SIDES["air"][2].split("/")[0], SIDES["ground"][2].split("/")[0]
    print(f"ping, ground {gnd_ip} -> aircraft {air_ip}:")
    c = connect(ground)
    out = sh(c, f"ping -c 10 -i 0.3 -W 2 -I {gnd_ip} {air_ip} | tail -2")
    c.close()
    print("\n".join("  " + l for l in out.splitlines()))
    print(f"TCP, aircraft -> ground, {secs} s:")
    tcp(ground, gnd_ip, air, air_ip, 5201, secs)
    print(f"TCP, ground -> aircraft, {secs} s:")
    tcp(air, air_ip, ground, gnd_ip, 5202, secs)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["on", "test", "off"])
    ap.add_argument("--air", required=True, help="the aircraft (transmitting) board's address")
    ap.add_argument("--ground", required=True, help="the ground (receiving) board's address")
    ap.add_argument("--secs", type=int, default=10, help="test: seconds of TCP each way (default 10)")
    a = ap.parse_args()
    if a.action == "test":
        test(a.air, a.ground, a.secs)
        return
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
