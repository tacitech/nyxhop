#!/usr/bin/env python3
"""Install the on-board camera app (nyx-ipcam) on a board, either kind, and start it.

    python <this file> <board ip> [--no-start]

The ARM program is taken from your own cross-build if there is one
(cargo build --release -p nyx-ipcam --target armv7-unknown-linux-gnueabihf), else from the
prebuilt one in deploy/ipcam/. NYX_IPCAM_BIN=<file> names another.

ADRV9364 (Kuiper, systemd): /home/root/nyx-ipcam + nyx-ipcam.service, enabled and started.
ANTSDR E200 (runs from RAM): nyx-ipcam, local.sh, ipcam-loop.sh and the start.sh with the
local.sh hook, into /tmp/nyxhop and onto the SD card (/nyxhop), then started.

The app follows the board's role by itself, so it goes on every board: on the receiving end
it waits. Authentication: the ssh key ~/.ssh/nyx_board if you have one, else the password in
BOARD_PW (set BOARD_PW=analog; PowerShell: $env:BOARD_PW = "analog").
"""
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))


def find(what, *cands):
    """The first of the candidate paths that exists (a development tree and the released
    tree keep the same files in different places)."""
    for c in cands:
        if c and os.path.exists(c):
            return c
    tried = [c for c in cands if c]
    sys.exit(f"missing {what}: looked in " + ", ".join(tried))


def connect(host):
    try:
        import paramiko
    except ImportError:
        sys.exit("need paramiko:  python -m pip install paramiko")
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    key = os.path.expanduser(os.environ.get("BOARD_KEY", "~/.ssh/nyx_board"))
    if os.path.exists(key):
        try:
            cli.connect(host, username="root", key_filename=key, timeout=10,
                        look_for_keys=False, allow_agent=False)
            return cli, "key"
        except Exception:
            pass
    pw = os.environ.get("BOARD_PW")
    if not pw:
        sys.exit("set BOARD_PW to the board's ssh password first")
    cli.connect(host, username="root", password=pw, look_for_keys=False, allow_agent=False, timeout=10)
    return cli, "password"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 1:
        sys.exit(__doc__)
    bin_path = find(
        "the ARM build of nyx-ipcam",
        os.environ.get("NYX_IPCAM_BIN"),
        os.path.join(ROOT, "target", "armv7-unknown-linux-gnueabihf", "release", "nyx-ipcam"),
        os.path.join(ROOT, "deploy", "ipcam", "nyx-ipcam"),
    )
    service = find("nyx-ipcam.service", os.path.join(HERE, "nyx-ipcam.service"),
                   os.path.join(ROOT, "deploy", "ipcam", "nyx-ipcam.service"))
    e200_dir = os.path.join(ROOT, "deploy", "ipcam", "e200")
    local_sh = find("local.sh", os.path.join(HERE, "e200", "local.sh"), os.path.join(e200_dir, "local.sh"))
    loop_sh = find("ipcam-loop.sh", os.path.join(HERE, "e200", "ipcam-loop.sh"), os.path.join(e200_dir, "ipcam-loop.sh"))
    start_sh = find("start.sh", os.path.join(ROOT, "board", "e200", "antsdr", "start.sh"),
                    os.path.join(ROOT, "deploy", "e200", "nyxhop", "start.sh"))
    print(f"program: {bin_path}")
    cli, how = connect(args[0])
    print(f"connected to {args[0]} ({how})")

    def sh(cmd):
        _, o, e = cli.exec_command(cmd, timeout=60)
        out = o.read().decode(errors="replace")
        err = e.read().decode(errors="replace")
        return out.strip(), err.strip()

    def put(data, remote, mode="755"):
        i, o, _ = cli.exec_command(f"cat > '{remote}'", timeout=300)
        i.write(data)
        i.channel.shutdown_write()
        o.channel.recv_exit_status()
        got, _ = sh(f"chmod {mode} '{remote}'; md5sum '{remote}'")
        want = hashlib.md5(data).hexdigest()
        if got.split()[:1] != [want]:
            sys.exit(f"upload of {remote} failed (md5 {got!r}, want {want})")
        print(f"  {remote}  {len(data)} B")

    def text(path):
        # the board's shell dies on CRLF (a Windows checkout)
        return open(path, "rb").read().replace(b"\r\n", b"\n")

    binary = open(bin_path, "rb").read()
    kuiper = sh("test -f /home/root/nyxhop-start.py && echo yes")[0] == "yes"
    e200 = sh("test -f /tmp/nyxhop/start.sh && echo yes")[0] == "yes"
    start = "--no-start" not in sys.argv

    if kuiper:
        print("ADRV9364 / Kuiper")
        sh("systemctl stop nyx-ipcam 2>/dev/null")
        put(binary, "/home/root/nyx-ipcam")
        put(text(service), "/etc/systemd/system/nyx-ipcam.service", "644")
        sh("systemctl daemon-reload; systemctl enable nyx-ipcam 2>&1")
        if start:
            sh("systemctl start nyx-ipcam")
        print("  service:", sh("systemctl is-enabled nyx-ipcam; systemctl is-active nyx-ipcam")[0].replace("\n", ", "))
    elif e200:
        print("ANTSDR E200 (RAM + SD card)")
        D = "/tmp/nyxhop"
        sh(f"[ -f {D}/local.sh ] && sh {D}/local.sh stop")
        files = {
            "nyx-ipcam": binary,
            "local.sh": text(local_sh),
            "ipcam-loop.sh": text(loop_sh),
            "start.sh": text(start_sh),
        }
        for name, data in files.items():
            put(data, f"{D}/{name}")
        names = " ".join(f"{D}/{n}" for n in files)
        out, err = sh(
            "umount /mnt/sd 2>/dev/null; mkdir -p /mnt/sd && mount /dev/mmcblk0p1 /mnt/sd && "
            f"mkdir -p /mnt/sd/nyxhop && cp {names} /mnt/sd/nyxhop/ && sync && "
            "md5sum /mnt/sd/nyxhop/nyx-ipcam; umount /mnt/sd"
        )
        if hashlib.md5(binary).hexdigest() not in out:
            sys.exit(f"copy to the SD card failed: {out} {err}")
        print("  SD card: /nyxhop updated")
        if start:
            sh(f"sh {D}/local.sh start")
        print("  running:", sh("pidof nyx-ipcam || echo no")[0])
    else:
        sys.exit("neither a Kuiper board (/home/root/nyxhop-start.py) nor an E200 (/tmp/nyxhop/start.sh)")
    cli.close()


if __name__ == "__main__":
    main()
