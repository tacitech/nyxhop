#!/usr/bin/env python3
"""Start both PC apps (nyx-rx and nyx-tx) and apply a video configuration.

The boards start their own radio daemon at boot, so this only looks after the PC side. The
apps reconnect to a board by themselves, so the order of powering things up does not matter.

    python nyx-client.py                                   # defaults below
    python nyx-client.py --rx-board 192.168.0.12:7011      # E200 as the receiving end
    python nyx-client.py --source pattern                  # test pattern instead of the camera

Run it by hand, or put it in the Windows Startup folder / Task Scheduler (at logon) for an
unattended ground station. Logs go to target/release/logs/.
"""
import argparse
import os
import socket
import subprocess
import sys
import time


def workspace_root():
    """Nearest ancestor holding the cargo workspace (this script moves between trees)."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        p = os.path.join(d, "Cargo.toml")
        if os.path.isfile(p) and "[workspace]" in open(p, encoding="utf-8").read():
            return d
        up = os.path.dirname(d)
        if up == d:
            sys.exit("no cargo workspace above " + __file__)
        d = up


ROOT = workspace_root()
REL = os.path.join(ROOT, "target", "release")
LOGD = os.path.join(REL, "logs")
EXE = ".exe" if os.name == "nt" else ""
TX = os.path.join(REL, "nyx-tx" + EXE)
RX = os.path.join(REL, "nyx-rx" + EXE)


def con(cmd, port, wait=0.3):
    """One line to an app's console, and its reply."""
    try:
        s = socket.create_connection(("127.0.0.1", port), timeout=3)
    except OSError:
        return ""
    s.sendall((cmd + "\n").encode())
    s.settimeout(2.0)
    time.sleep(wait)
    d = b""
    try:
        while b"ok" not in d and b"err" not in d:
            d += s.recv(4096)
    except OSError:
        pass
    s.close()
    return d.decode(errors="replace")


def launch(exe, args, log):
    os.makedirs(LOGD, exist_ok=True)
    # Detached from this console (0x8 on Windows), working directory beside the executables
    # so logs/ lands there.
    flags = 0x00000008 if os.name == "nt" else 0
    return subprocess.Popen(
        [exe, *args],
        cwd=REL,
        stdout=open(os.path.join(LOGD, log), "w"),
        stderr=subprocess.STDOUT,
        creationflags=flags,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tx-board", default="192.168.0.10:7010", help="transmitting end's board")
    ap.add_argument("--rx-board", default="192.168.0.11:7011", help="receiving end's board")
    ap.add_argument("--source", default="webcam", choices=["webcam", "pattern"])
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--quality", type=int, default=60)
    a = ap.parse_args()
    vcfg = [f"set source {a.source}", "set codec h264", f"set fps {a.fps}",
            f"set quality {a.quality}", "set pair 1", "set auto 1", "set arq 1"]

    # 1) whatever is still running from last time
    if os.name == "nt":
        subprocess.run(["powershell", "-NoProfile", "-Command",
                        "Stop-Process -Name nyx-tx,nyx-rx -Force -ErrorAction SilentlyContinue"],
                       capture_output=True)
    else:
        subprocess.run(["pkill", "-x", "nyx-tx"], capture_output=True)
        subprocess.run(["pkill", "-x", "nyx-rx"], capture_output=True)
    time.sleep(1)

    # 2) both apps; they reconnect to their boards on their own
    launch(RX, ["--channel", a.rx_board, "--ctl", "127.0.0.1:7203"], "nyx-rx.log")
    launch(TX, ["--channel", a.tx_board, "--ctl", "127.0.0.1:7204"], "nyx-tx.log")
    print("apps launched, waiting for their consoles...", flush=True)

    # 3) until both consoles answer
    for _ in range(30):
        if "source" in con("get", 7204) and "stats" in con("stats", 7203) + "stats":
            break
        time.sleep(1)

    # 4) the video configuration, with a few retries while the app settles
    for c in vcfg:
        ok = False
        for _ in range(5):
            if "ok" in con(c, 7204):
                ok = True
                break
            time.sleep(1)
        print(f"  {c} -> {'ok' if ok else 'FAIL'}", flush=True)
    print("DONE. The apps are running; video appears once the boards are up and paired.")


if __name__ == "__main__":
    main()
