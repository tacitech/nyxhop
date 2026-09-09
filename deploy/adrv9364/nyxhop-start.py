#!/usr/bin/env python3
"""nyxhop-start - start the NyxHop radio daemon on ADI's stock Kuiper Linux image.

Run by systemd (nyxhop.service). It:
  1. waits for eth0 to have an address; reads the role from /home/root/nyx-role
     (written by the installer), falling back to the address (.10 = A, .11 = B)
  2. cho phy AD9361 (iio device co in_voltage_rf_bandwidth); dat gain manual +
     sets manual gain and the LOs through sysfs BEFORE the daemon starts, so nothing
     competes for the SPI bus and the receiver does not sit at maximum gain
  3. tim uio "dma" (--uio) va buffer capture: /dev/udmabuf0 (module u-dma-buf)
     or the uio "capbuf" device (reserved memory, no module needed)
  4. starts the daemon, logging to /home/root/radio.out
  5. applies /home/root/nyx-radio.cfg through the console on port 7202, retrying each line
  6. o lai giu daemon (systemd Restart=on-failure keo ca cum len khi chet)
"""
import glob
import os
import socket
import subprocess
import sys
import time

HOME = "/home/root"
CFG = f"{HOME}/nyx-radio.cfg"
ROLE_FILE = f"{HOME}/nyx-role"
DAEMON = f"{HOME}/nyx-radio-node"
LOG = f"{HOME}/radio.out"
ROLES = {
    "A": dict(ip="192.168.0.10", rxlo=5835000000, txlo=5745000000, gain=62),
    "B": dict(ip="192.168.0.11", rxlo=5745000000, txlo=5835000000, gain=60),
}
BASE_ARGS = ("--fir {home}/fir10MHz.ftr --mod-base 0x43C30000 --demod-base 0x43C40000 "
             "--trig-base 0x43C00000 --agc-base 0x43C60000 --video-hz 5745000000")


def log(s):
    print(f"nyxhop: {s}", flush=True)


def eth_ip(timeout=40):
    for _ in range(timeout):
        out = subprocess.run(["ip", "-4", "-o", "addr", "show", "eth0"], capture_output=True, text=True).stdout
        for tok in out.split():
            if "." in tok and "/" in tok:
                return tok.split("/")[0]
        time.sleep(1)
    return ""


def find_phy(timeout=40):
    for _ in range(timeout):
        for d in glob.glob("/sys/bus/iio/devices/iio:device*"):
            if os.path.exists(f"{d}/in_voltage_rf_bandwidth"):
                return d
        time.sleep(1)
    return ""


def uio_by_name():
    m = {}
    for d in glob.glob("/sys/class/uio/uio*"):
        try:
            m[open(f"{d}/name").read().strip()] = os.path.basename(d)
        except OSError:
            pass
    return m


def console(line, wait=3.0):
    try:
        s = socket.create_connection(("127.0.0.1", 7202), timeout=3)
    except OSError:
        return ""
    try:
        s.sendall((line + "\n").encode())
        s.settimeout(wait)
        d = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            try:
                c = s.recv(4096)
            except OSError:
                break
            if not c:
                break
            d += c
            if d.rstrip().endswith(b"ok") or b"err" in d:
                break
        return d.decode(errors="replace")
    finally:
        s.close()


def cfg(line):
    for _ in range(8):
        if "ok" in console(line):
            return True
        time.sleep(3)
    log(f"cfg FAIL: {line}")
    return False


def main():
    ip = eth_ip()
    role = ""
    if os.path.exists(ROLE_FILE):
        role = open(ROLE_FILE).read().strip().lower()
        role = {"tx": "A", "rx": "B", "a": "A", "b": "B"}.get(role, "")
    if role not in ROLES:
        role = next((r for r, v in ROLES.items() if v["ip"] == ip), "")
    if role not in ROLES:
        log(f"address '{ip}' is not A/B and there is no {ROLE_FILE} - stopping")
        sys.exit(1)
    r = ROLES[role]
    log(f"role={role} ip={ip} - starting")

    phy = find_phy()
    if not phy:
        log("AD9361 phy not found (has the ad9361-phy driver come up?)")
        sys.exit(1)
    for attr, val in (("in_voltage0_gain_control_mode", "manual"),
                      ("in_voltage0_hardwaregain", r["gain"]),
                      ("out_altvoltage0_RX_LO_frequency", r["rxlo"]),
                      ("out_altvoltage1_TX_LO_frequency", r["txlo"])):
        try:
            open(f"{phy}/{attr}", "w").write(str(val))
        except OSError as e:
            log(f"sysfs {attr}: {e}")

    # v40.26: module u-dma-buf build rieng cho kernel Kuiper (board/kuiper-2023_r2/
    # If the module is present, load it for a CACHED capture buffer (about three times
    # faster per sample than the uncached uio one); otherwise fall back to uio capbuf.
    ko = f"{HOME}/u-dma-buf.ko"
    if os.path.exists(ko) and not os.path.exists("/dev/udmabuf0"):
        ir = subprocess.run(["insmod", ko], capture_output=True, text=True)
        log(f"insmod u-dma-buf.ko: rc={ir.returncode} {ir.stderr.strip()[:120]}")
        time.sleep(1)
    uio = uio_by_name()
    if "dma" not in uio:
        log(f"uio 'dma' not found (have: {uio}) - is the DMA node missing from the device tree?")
        sys.exit(1)
    if os.path.exists("/dev/udmabuf0"):
        capbuf = "udmabuf0"
    elif "capbuf" in uio:
        capbuf = f"uio:{uio['capbuf']}"
    else:
        log("neither /dev/udmabuf0 nor uio 'capbuf' - no capture buffer")
        sys.exit(1)
    args = f"--uio {uio['dma']} --udmabuf {capbuf} " + BASE_ARGS.format(home=HOME)
    log(f"daemon: {DAEMON} {args}")

    subprocess.run(["killall", "nyx-radio-node"], capture_output=True)
    time.sleep(1)
    out = open(LOG, "ab")
    proc = subprocess.Popen([DAEMON] + args.split(), cwd=HOME, stdout=out, stderr=subprocess.STDOUT)

    if os.path.exists(CFG):
        for line in open(CFG, encoding="utf-8", errors="replace"):
            line = line.split("#")[0].strip()
            if line:
                cfg(line)
    else:
        log(f"no {CFG} - falling back to minimal LO/gain")
        for line in (f"rxfreq {r['rxlo']}", f"txfreq {r['txlo']}", "txgain 0", "agc manual", f"rxgain {r['gain']}"):
            cfg(line)
    log(f"{role} READY")
    rc = proc.wait()
    log(f"daemon exited rc={rc}")
    sys.exit(1 if rc else 0)


if __name__ == "__main__":
    main()
