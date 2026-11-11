#!/usr/bin/env python3
"""Put the flight controller on a NyxHop board, take it off again, or see what runs there.

    python install.py install <board address>
    python install.py uninstall <board address> [--all]
    python install.py status <board address>

The board must already run the NyxHop link (link/scripts/nyx_flash.py) and the camera app
(link/scripts/nyx_ipcam_install.py), which carries PX4's MAVLink over the link.

install uploads the board image in deploy/: the FPGA image with the flight-controller I/O, the
Linux kernel with CAN and the board's UARTs, the device tree and PX4. It checks every file after
the upload, reboots the board and reads back what runs. A file it replaces stays beside the new one
with .pre-fc added to its name, and uninstall puts it back. Nothing is built here, and the radio's
settings (channels, pairing, licence, antenna port) are left as they are.

uninstall keeps PX4's parameters and logs in /home/root/px4 unless you add --all.

Needs Python 3 and paramiko (python -m pip install paramiko). It logs in as root with the key
~/.ssh/nyx_board if you have one (BOARD_KEY names another), else with the password in BOARD_PW
(set BOARD_PW=analog; PowerShell: $env:BOARD_PW = "analog").
"""
import argparse
import hashlib
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HOME = "/home/root"
PX4 = HOME + "/px4"
RECORD = PX4 + "/installed.md5"
UNIT = "/etc/systemd/system/px4.service"
DROPIN = "/etc/systemd/system/nyx-ipcam.service.d/px4-tlm.conf"
# the files the image replaces on the board; each one's old version is kept as <name>.pre-fc
REPLACED = [
    ("BOOT.BIN", "/boot/BOOT.BIN"),
    ("uImage", "/boot/uImage"),
    ("devicetree.dtb", "/boot/devicetree.dtb"),
    ("u-dma-buf.ko", HOME + "/u-dma-buf.ko"),
]
SCRIPTS = ["fc-start.sh", "fc-status.sh"]
TEXT = (".sh", ".service", ".conf", ".config")


def md5(data):
    return hashlib.md5(data).hexdigest()


class Payload:
    """deploy/: its MANIFEST and the files, each checked against the list"""

    def __init__(self, path):
        self.path = path
        manifest = os.path.join(path, "MANIFEST")
        if not os.path.isfile(manifest):
            sys.exit(f"{path} holds no board image (no MANIFEST): pass --payload <folder>")
        self.info, self.sums = {}, {}
        for line in open(manifest, encoding="utf-8").read().splitlines():
            parts = line.split()
            if len(parts) == 3 and len(parts[0]) == 32 and parts[1].isdigit():
                self.sums[parts[2]] = parts[0]
            elif ": " in line:
                key, value = line.split(": ", 1)
                self.info[key.strip()] = value.strip()
        self.files = {}
        for name, want in self.sums.items():
            try:
                data = open(os.path.join(path, name), "rb").read()
            except OSError:
                sys.exit(f"{path}: {name} is missing")
            # (a Windows checkout may have turned the scripts' line ends into CRLF)
            if name.endswith(TEXT):
                data = data.replace(b"\r\n", b"\n")
            if md5(data) != want:
                sys.exit(f"{path}: {name} is not the file MANIFEST lists (a broken download?)")
            self.files[name] = data


class Board:
    def __init__(self, host):
        import warnings
        warnings.filterwarnings("ignore")
        try:
            import paramiko
        except ImportError:
            sys.exit("need paramiko:  python -m pip install paramiko")
        self.host = host
        self.cli = paramiko.SSHClient()
        self.cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        key = os.path.expanduser(os.environ.get("BOARD_KEY", "~/.ssh/nyx_board"))
        if os.path.exists(key):
            try:
                self.cli.connect(host, username="root", key_filename=key, timeout=10,
                                 look_for_keys=False, allow_agent=False)
                return
            except Exception:
                pass
        pw = os.environ.get("BOARD_PW")
        if not pw:
            raise OSError("no ssh key that the board takes, and BOARD_PW is not set")
        self.cli.connect(host, username="root", password=pw, timeout=10,
                         look_for_keys=False, allow_agent=False)

    def sh(self, cmd, timeout=120):
        _, out, err = self.cli.exec_command(cmd, timeout=timeout)
        text = out.read().decode(errors="replace") + err.read().decode(errors="replace")
        return text.strip()

    def ok(self, test):
        return self.sh(f"{test} && echo yes") == "yes"

    def md5(self, path):
        return self.sh(f"md5sum '{path}' 2>/dev/null | cut -c1-32")

    def put(self, data, remote, mode="644"):
        _, out, err = self.cli.exec_command(f"cat > '{remote}'", timeout=600)
        out.channel.sendall(data)
        out.channel.shutdown_write()
        if out.channel.recv_exit_status() != 0:
            sys.exit(f"upload of {remote} failed: {err.read().decode(errors='replace')[:200]}")
        if self.md5(remote) != md5(data):
            sys.exit(f"upload of {remote} failed: the board's copy differs")
        self.sh(f"chmod {mode} '{remote}'")

    def close(self):
        self.cli.close()


def connect(host):
    try:
        return Board(host)
    except Exception as e:
        sys.exit(f"cannot log in to {host}: {e}")


def ssh_port_open(host):
    try:
        socket.create_connection((host, 22), timeout=2).close()
        return True
    except OSError:
        return False


def reboot(b):
    """the board restarted and logged in to again; None when it does not come back"""
    host = b.host
    print(f"rebooting {host} ...")
    b.sh("sync; (sleep 1; systemctl reboot) > /dev/null 2>&1 &", timeout=10)
    b.close()
    t0 = time.time()
    while ssh_port_open(host) and time.time() - t0 < 60:
        time.sleep(1)
    while time.time() - t0 < 240:
        if ssh_port_open(host):
            try:
                b = Board(host)
                print(f"  back after {int(time.time() - t0)} s")
                return b
            except Exception:
                pass
        time.sleep(3)
    print(f"  {host} did not come back in 240 s. If your network gave it another address, run\n"
          f"  'install.py status <new address>'; else look at its serial console.")
    return None


def read_status(b):
    out = b.sh(f"sh {PX4}/fc-status.sh", timeout=60)
    return dict(line.split("=", 1) for line in out.splitlines() if "=" in line)


def radio_alone(b):
    """the radio without the flight controller's status script (after an uninstall)"""
    active = b.sh("systemctl is-active nyxhop")
    answers = "tx run=" in b.sh(f"python3 {HOME}/nyxctl.py o64 2>/dev/null")
    return active, answers


def show(host, st, want_fcio=None):
    """print what fc-status.sh read; -> the list of things that are not as they should be"""
    bad = []
    fc = st.get("image") == "fc"
    print(f"{host}: the flight controller is installed")
    if fc:
        line = f"flight-controller image, FC I/O {st.get('fcio', '?')}"
        if want_fcio and st.get("fcio", "").lower() != want_fcio.lower():
            bad.append(f"the FC I/O block reads {st.get('fcio')}, the image's is {want_fcio}")
    else:
        line = "not the flight-controller image (was the link flashed again? run install once more)"
        bad.append("/boot/BOOT.BIN is not the flight-controller image")
    print(f"  FPGA image     {line}")
    print(f"  Linux kernel   {st.get('kernel', '?')}")

    radio = st.get("nyxhop", "?")
    if radio == "active" and st.get("radio_role"):
        heard = "ground station heard" if int(st.get("radio_heard") or 0) > 0 else \
            "no ground station heard (is it on?)"
        line = f"running (role {st['radio_role']}), licence: {st.get('licence') or '?'}, {heard}"
    else:
        line = f"{radio}, its console does not answer"
        bad.append("the radio does not run")
    print(f"  radio          {line}")
    if st.get("nyx-ipcam") != "active":
        bad.append("the camera app (nyx-ipcam) does not run: no MAVLink over the link")
        print(f"  camera app     {st.get('nyx-ipcam', '?')}")

    px4 = st.get("px4", "?")
    print(f"  PX4            {'running' if px4 == 'active' else px4}, mode {st.get('mode') or '?'}")
    if px4 != "active":
        bad.append("PX4 does not run")
        for l in (st.get("px4_log") or "").split("|"):
            if l.strip():
                print(f"                 {l.strip()}")
    else:
        if st.get("imu_z"):
            errors = int(st.get("imu_errors") or 0)
            print(f"  IMU            answers: z {float(st['imu_z']):+.2f} m/s2, {float(st.get('imu_temp') or 0):.0f} C"
                  + (f", {errors} read error{'' if errors == 1 else 's'} since PX4 started" if errors else ""))
        else:
            print("  IMU            no data")
            bad.append("the IMU gives no data")
        if fc:
            if st.get("failsafe") == "0":
                print("  motor outputs  written by PX4")
            else:
                print("  motor outputs  held at stop by the FPGA: PX4 is not writing them")
                bad.append("PX4 does not write the motor outputs")
    ports = (st.get("ports") or "").split()
    print(f"  ports          {' '.join(ports) or 'none'}")
    if st.get("mode") == "bb":
        missing = [n for n in ("imu", "rc", "telem1", "gps2", "gps1") if not any(p.startswith(n + ":") for p in ports)]
        if missing:
            bad.append(f"ports not up: {', '.join(missing)}")
    return bad


def settled(st):
    return st.get("px4") == "active" and st.get("radio_role") and st.get("imu_z") and st.get("failsafe") == "0"


def install(a):
    p = Payload(a.payload)
    b = connect(a.host)
    host = a.host
    link = p.info.get("link", "?")

    # is this a board the image is for?
    if not b.ok(f"test -f {HOME}/nyxhop-start.py -a -f {HOME}/nyx-radio-node-o64 -a -f /boot/BOOT.BIN"):
        sys.exit(f"{host} does not run the NyxHop link: flash it first (link/scripts/nyx_flash.py)")
    if not b.ok(f"test -f {HOME}/nyx-ipcam -a -f /etc/systemd/system/nyx-ipcam.service"):
        sys.exit(f"{host} has no camera app, and PX4's MAVLink goes over the link through it:\n"
                 "install it first (link/scripts/nyx_ipcam_install.py), a camera is not needed")
    problems = []
    if b.md5(f"{HOME}/nyx-radio-node-o64") != p.info.get("link-daemon"):
        problems.append(f"the board's link is not release {link}, which this image is made for: "
                        "flash that release first (link/scripts/nyx_flash.py)")
    release = b.sh("uname -r")
    if release != p.info.get("kernel-release"):
        problems.append(f"the board's Linux is {release}; the image's kernel is "
                        f"{p.info.get('kernel-release')} and the board's modules would not load in it")
    if problems and not a.force:
        sys.exit("not installed:\n  " + "\n  ".join(problems) + "\n(--force installs anyway)")
    free = b.sh("df -Pm /boot | awk 'NR == 2 { print $4 }'")
    if free.isdigit() and int(free) < 40:
        sys.exit(f"/boot has {free} MB free; the image and the files it keeps need 40 MB")

    print(f"{host}: installing the flight controller (link {link}, PX4 {p.info.get('px4', '?')})")
    if b.sh("systemctl is-active px4") == "active":
        b.sh("systemctl stop px4")
        print("  PX4 stopped (the FPGA holds the motor outputs at stop)")

    # the files of the board the image replaces: uploaded beside them first, swapped in together
    ours = {tuple(l.split()) for l in b.sh(f"cat {RECORD} 2>/dev/null").splitlines() if len(l.split()) == 2}
    swap = []
    for name, path in REPLACED:
        want = p.sums[name]
        have = b.md5(path)
        if have == want:
            print(f"  {path}: already the image's")
            continue
        b.put(p.files[name], path + ".fc-new")
        # what is there now is the board's own unless an earlier install wrote it: kept
        keep = bool(have) and (have, path) not in ours
        swap.append((path, keep))
    for path, keep in swap:
        cmd = f"mv '{path}.fc-new' '{path}'"
        if keep:
            cmd = f"cp -p '{path}' '{path}.pre-fc' && " + cmd
        out = b.sh(cmd + " && sync && echo done")
        if out != "done":
            sys.exit(f"{path}: {out}")
        print(f"  {path}: written" + (f" (the board's own kept as {os.path.basename(path)}.pre-fc)" if keep else ""))

    # PX4 and its start files
    b.sh(f"mkdir -p {PX4}")
    b.put(p.files["px4.tar.gz"], f"{PX4}/px4.tar.gz")
    out = b.sh(f"cd {PX4} && rm -rf bin etc && tar xzf px4.tar.gz && rm px4.tar.gz && test -x bin/px4 && echo done")
    if out != "done":
        sys.exit(f"unpacking PX4 failed: {out}")
    for name in SCRIPTS:
        b.put(p.files[name], f"{PX4}/{name}", "755")
    for name in sorted(n for n in p.files if n.endswith(".dtbo")):
        b.put(p.files[name], f"{PX4}/{name}")
    b.put(b"bb\n", f"{PX4}/mode")
    b.put((p.sums["BOOT.BIN"] + "\n").encode(), f"{PX4}/fc-bootbin.md5")
    # (what this and earlier installs wrote: a later one must not keep those as the board's own)
    ours |= {(p.sums[name], path) for name, path in REPLACED}
    b.put("".join(f"{s} {path}\n" for s, path in sorted(ours)).encode(), RECORD)
    print(f"  {PX4}: PX4, its start files and the ports' device tree overlays")

    # PX4 at every boot, MAVLink between it and the link
    b.put(p.files["px4.service"], UNIT)
    b.sh(f"mkdir -p {os.path.dirname(DROPIN)}")
    b.put(p.files["px4-tlm.conf"], DROPIN)
    out = b.sh("systemctl daemon-reload && systemctl enable px4 2>&1 && sync && echo done")
    if not out.endswith("done"):
        sys.exit(f"enabling px4.service failed: {out}")
    print("  px4.service enabled; the camera app hands the link's MAVLink to PX4")

    b = reboot(b)
    if b is None:
        sys.exit(1)
    bad = [f"{path} on the card is not the image's file" for name, path in REPLACED if b.md5(path) != p.sums[name]]
    st, up = {}, None
    t0 = time.time()
    while time.time() - t0 < 150:
        st = read_status(b)
        if settled(st):
            up = up or time.time()
            # (the two ends of the link find each other some seconds after the radio starts)
            if int(st.get("radio_heard") or 0) > 0 or time.time() - up > 40:
                break
        time.sleep(5)
    bad += show(host, st, p.info.get("fc-io"))
    b.close()
    if bad:
        print("\nNOT RIGHT:\n  " + "\n  ".join(bad))
        sys.exit(1)
    print("\nInstalled. PX4 starts with the board from now on; "
          "calibrate and set it up from the ground station as any PX4.")


def uninstall(a):
    b = connect(a.host)
    host = a.host
    if not b.ok(f"test -e {UNIT} -o -e {PX4}/fc-start.sh -o -e {RECORD}"):
        sys.exit(f"{host}: the flight controller is not installed")
    print(f"{host}: removing the flight controller")
    b.sh(f"systemctl disable --now px4 2>/dev/null; rm -f {UNIT} {DROPIN}; "
         f"rmdir {os.path.dirname(DROPIN)} 2>/dev/null; systemctl daemon-reload")
    print("  PX4 stopped, px4.service removed, the camera app as it was")
    ours = {tuple(l.split()) for l in b.sh(f"cat {RECORD} 2>/dev/null").splitlines() if len(l.split()) == 2}
    for name, path in REPLACED:
        short = os.path.basename(path)
        if not b.ok(f"test -e '{path}.pre-fc'"):
            print(f"  {path}: left (nothing was kept for it: it was the image's file before the install)")
        elif (b.md5(path), path) not in ours:
            print(f"  {path}: left, it changed after the install (the older file stays as {short}.pre-fc)")
        else:
            out = b.sh(f"mv '{path}.pre-fc' '{path}' && sync && echo done")
            if out != "done":
                sys.exit(f"{path}: {out}")
            print(f"  {path}: the board's own file is back")
    if a.all:
        b.sh(f"rm -rf {PX4}")
        print(f"  {PX4} removed")
    else:
        b.sh(f"cd {PX4} && rm -rf bin etc nyxhop-*.config {' '.join(SCRIPTS)} *.dtbo mode fc-bootbin.md5 "
             f"{os.path.basename(RECORD)} px4.tar.gz; cd / && rmdir {PX4} 2>/dev/null")
        if b.ok(f"test -d {PX4}"):
            print(f"  PX4's parameters and logs stay in {PX4} (uninstall --all removes them too)")
    b = reboot(b)
    if b is None:
        sys.exit(1)
    active, answers = "", False
    t0 = time.time()
    while time.time() - t0 < 90 and not answers:
        active, answers = radio_alone(b)
        if not answers:
            time.sleep(5)
    b.close()
    print(f"{host}: the flight controller is removed; the radio is {active}"
          + ("" if answers else ", but its console does not answer"))
    if not answers:
        sys.exit(1)


def status(a):
    b = connect(a.host)
    if not b.ok(f"test -f {PX4}/fc-status.sh"):
        active, answers = radio_alone(b)
        print(f"{a.host}: the flight controller is not installed; the radio is {active}"
              + ("" if answers else ", its console does not answer"))
        b.close()
        return
    want = None
    if os.path.isfile(os.path.join(a.payload, "MANIFEST")):
        want = Payload(a.payload).info.get("fc-io")
    bad = show(a.host, read_status(b), want)
    b.close()
    if bad:
        print("\nNOT RIGHT:\n  " + "\n  ".join(bad))
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["install", "uninstall", "status"])
    ap.add_argument("host", metavar="board-address")
    ap.add_argument("--payload", default=os.path.join(HERE, "deploy"),
                    help="the board image (default: deploy/ beside this file)")
    ap.add_argument("--force", action="store_true",
                    help="install on a board whose link release or Linux is not the image's")
    ap.add_argument("--all", action="store_true", help="uninstall: PX4's parameters and logs too")
    a = ap.parse_args()
    {"install": install, "uninstall": uninstall, "status": status}[a.command](a)


if __name__ == "__main__":
    main()
