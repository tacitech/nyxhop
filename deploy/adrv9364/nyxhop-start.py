#!/usr/bin/env python3
"""nyxhop-start - start the NyxHop radio daemon on ADI's stock Kuiper Linux image.

Run by systemd (nyxhop.service). It:
  1. waits for eth0 to have an address; reads the role from /home/root/nyx-role
     (written by the installer), falling back to the address (.10 = A, .11 = B)
  2. waits for the AD9361 phy (the iio device with in_voltage_rf_bandwidth), then
     sets manual gain and the LOs through sysfs BEFORE the daemon starts, so nothing
     competes for the SPI bus and the receiver does not sit at maximum gain
  3. finds the uio "dma" (--uio) and the capture buffer: /dev/udmabuf0 (module u-dma-buf)
     or the uio "capbuf" device (reserved memory, no module needed)
  4. starts the daemon, logging to /home/root/radio.out; extra daemon flags (one line,
     e.g. `--tx-ring 32768` for a bench sweep) are read from /home/root/nyx-args
  5. applies /home/root/nyx-radio.cfg through the console on port 7202, retrying each line
  6. stays with the daemon (systemd Restart=on-failure brings the lot back if it dies)

Branch B (the ofdm64 PHY, bitstream B in BOOT.BIN): /home/root/nyx-phy64 present -> the phy64
daemon /home/root/nyx-radio-node-o64 with its state dir /home/root/o64 (role, cfg, tables, pairing
there; `role tx|rx` from an app rewrites that dir's nyx-radio.cfg), logging to /home/root/o64.out.
"""
import glob
import os
import socket
import subprocess
import sys
import time

HOME = "/home/root"
CFG = f"{HOME}/nyx-radio.cfg"
ARGS_FILE = f"{HOME}/nyx-args"
ROLE_FILE = f"{HOME}/nyx-role"
DAEMON = f"{HOME}/nyx-radio-node"
LOG = f"{HOME}/radio.out"
PHY64_FLAG = f"{HOME}/nyx-phy64"
O64_DIR = f"{HOME}/o64"
O64_DAEMON = f"{HOME}/nyx-radio-node-o64"
O64_LOG = f"{HOME}/o64.out"
ROLES = {
    "A": dict(ip="192.168.0.10", rxlo=5835000000, txlo=5745000000, gain=62),
    "B": dict(ip="192.168.0.11", rxlo=5745000000, txlo=5835000000, gain=60),
}
BASE_ARGS = ("--fir {home}/fir10MHz.ftr --mod-base 0x43C30000 --demod-base 0x43C40000 "
             "--trig-base 0x43C00000 --agc-base 0x43C60000 --video-hz 5745000000")
# `role tx|rx` from an app: the daemon rewrites nyx-radio.cfg and runs this to restart. A
# transient systemd timer does it from outside our cgroup, so the request survives the
# service being stopped. Passed as one argument (args.split() would cut it up).
RESTART_CMD = "systemd-run --on-active=2 --timer-property=AccuracySec=100ms /bin/systemctl restart nyxhop"


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
    phy64 = os.path.exists(PHY64_FLAG)
    role_file = f"{O64_DIR}/nyx-role" if phy64 and os.path.exists(f"{O64_DIR}/nyx-role") else ROLE_FILE
    role = ""
    if os.path.exists(role_file):
        role = open(role_file).read().strip().lower()
        role = {"tx": "A", "rx": "B", "a": "A", "b": "B"}.get(role, "")
    if role not in ROLES:
        role = next((r for r, v in ROLES.items() if v["ip"] == ip), "")
    if role not in ROLES:
        log(f"address '{ip}' is not A/B and there is no {ROLE_FILE} - stopping")
        sys.exit(1)
    r = ROLES[role]
    log(f"role={role} ip={ip} phy={'ofdm64' if phy64 else 'ofdm1024'} - starting")

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

    # v40.26: the u-dma-buf module built for the Kuiper kernel (board/kuiper-2023_r2/
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
    if phy64:
        daemon, log_p, cfg_p, args_p = O64_DAEMON, O64_LOG, f"{O64_DIR}/nyx-radio.cfg", f"{O64_DIR}/nyx-args"
        args = f"--phy64 --uio {uio['dma']} --udmabuf {capbuf} --agc-base 0x43C60000 --state-dir {O64_DIR} --mode both"
    else:
        daemon, log_p, cfg_p, args_p = DAEMON, LOG, CFG, ARGS_FILE
        args = f"--uio {uio['dma']} --udmabuf {capbuf} " + BASE_ARGS.format(home=HOME)
    if os.path.exists(args_p):
        extra = " ".join(l.split("#")[0].strip() for l in open(args_p, encoding="utf-8", errors="replace"))
        if extra.strip():
            args += " " + extra.strip()
    log(f"daemon: {daemon} {args}")

    subprocess.run(["killall", "nyx-radio-node", "nyx-radio-node-o64"], capture_output=True)
    time.sleep(1)
    out = open(log_p, "ab")
    proc = subprocess.Popen([daemon] + args.split() + ["--restart-cmd", RESTART_CMD],
                            cwd=HOME, stdout=out, stderr=subprocess.STDOUT)

    if os.path.exists(cfg_p):
        for line in open(cfg_p, encoding="utf-8", errors="replace"):
            line = line.split("#")[0].strip()
            if line:
                cfg(line)
    else:
        log(f"no {cfg_p} - falling back to minimal LO/gain")
        for line in (f"rxfreq {r['rxlo']}", f"txfreq {r['txlo']}", "txgain 0", "agc manual", f"rxgain {r['gain']}"):
            cfg(line)
    log(f"{role} READY")
    rc = proc.wait()
    log(f"daemon exited rc={rc}")
    sys.exit(1 if rc else 0)


if __name__ == "__main__":
    main()
