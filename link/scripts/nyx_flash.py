#!/usr/bin/env python3
"""nyx_flash — write a prebuilt NyxHop release onto a board and verify it.

This tool does NOT build anything. It uploads the ready-made FPGA image and the radio
daemon from the release payload to the board over SSH, sets the board's role,
reboots it and checks it came back. You do not need Vivado, a cross-compiler, bootgen or
any build tool: the boot image, the FPGA design and the daemon are binaries in the payload.

  set BOARD_PW=analog                 # ssh password (PowerShell: $env:BOARD_PW = "analog")
  python nyx_flash.py --host <ip> --role tx            # ADRV9364 as the transmitting end
  python nyx_flash.py --host <ip> --role rx            # ADRV9364 as the receiving end
  python nyx_flash.py --host 192.168.0.12 --role rx --board e200
  python nyx_flash.py --host 192.168.0.10 --role tx --verify-only
  python nyx_flash.py --scan                          # find boards, address forgotten
  (options: --board adrv9364|e200, --payload <dir> (default ./bin),
   --mac <MAC>, --dry-run)

Payload layout (the committed deploy/ folder of the repository, per board type):
  deploy/adrv9364/  BOOT.BIN  devicetree.dtb  nyx-radio-node  fir10MHz.ftr
                    nyxhop-start.py  nyxctl.py  [u-dma-buf.ko]  nyxhop.service
                    nyx-radio-A.cfg  nyx-radio-B.cfg
  deploy/e200/      nyx.bit  uEnv.txt  uramdisk.image.gz  nyxhop/  nyx-radio-B-e200.cfg
"""
import argparse
import hashlib
import os
import socket
import subprocess
import sys
import time

CONSOLE_PORT = 7202  # every board answers here, which is how --scan recognises one
ROLE_ALIAS = {"tx": "A", "rx": "B", "a": "A", "b": "B"}


def norm_role(r):
    k = ROLE_ALIAS.get(str(r).lower())
    if not k:
        sys.exit(f"--role {r!r} invalid: tx | rx (or A | B)")
    return k


def md5(b):
    return hashlib.md5(b).hexdigest()


def drop_old_role_mode(b, home, role):
    """The channel mode a board remembers (nyx-hopmode.txt) names the role it was chosen
    under. Flashing the board into the other role must not replay it: an E200 moved from
    receiver to transmitter came up as a second receiver that way (10/9). Tables, pairing
    and licence stay; only the mode goes, and the new cfg sets it again."""
    old = b.sh(f"cat {home}/nyx-role 2>/dev/null").strip().upper()
    if old and old != role:
        b.sh(f"rm -f {home}/nyx-hopmode.txt")
        print(f"  role {old} -> {role}: the remembered channel mode is dropped (it named the old role)")


# ------------------------------------------------------------------- finding ----
def console(host, cmd, timeout=2.0):
    """One line to a board's console, and its reply. Empty when nothing answers."""
    try:
        s = socket.create_connection((host, CONSOLE_PORT), timeout=timeout)
    except OSError:
        return ""
    try:
        s.sendall((cmd + "\n").encode())
        s.settimeout(timeout)
        buf, t0 = b"", time.time()
        while time.time() - t0 < timeout:
            try:
                chunk = s.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            if buf.rstrip().endswith(b"ok") or b"err" in buf:
                break
        return buf.decode(errors="replace")
    finally:
        s.close()


def local_nets():
    """The /24 networks this computer sits on, so --scan needs no argument."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.0.2.1", 9))  # reserved address, nothing is sent
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    nets = []
    for ip in sorted(ips):
        if ip.startswith("127."):
            continue
        net = ip.rsplit(".", 1)[0]
        if net not in nets:
            nets.append(net)
    return nets


def scan(nets, timeout=0.4):
    """Knock on the console port across each /24 and ask whatever answers who it is."""
    from concurrent.futures import ThreadPoolExecutor

    def probe(host):
        try:
            s = socket.create_connection((host, CONSOLE_PORT), timeout=timeout)
            s.close()
            return host
        except OSError:
            return None

    hosts = [f"{net}.{i}" for net in nets for i in range(1, 255)]
    print(f"scanning {', '.join(n + '.0/24' for n in nets)} on port {CONSOLE_PORT} ...")
    with ThreadPoolExecutor(max_workers=256) as pool:
        found = [h for h in pool.map(probe, hosts) if h]
    if not found:
        print("no board answered. Is it powered, on this network, and given 20 s to start?")
        print("If your network is not one of the above, pass it: --scan 192.168.1.0/24")
        return found
    print(f"{'address':<16} {'role':<6} {'mode':<8} fpga")
    for host in found:
        st = console(host, "hop status")
        role = mode = ver = "?"
        for tok in st.split():
            if tok.startswith("hop_role="):
                role = tok.split("=", 1)[1]
            elif tok.startswith("hop_mode="):
                mode = tok.split("=", 1)[1]
        d = console(host, "demod")
        for tok in d.split():
            if tok.startswith("ver="):
                ver = tok.split("=", 1)[1]
        print(f"{host:<16} {role:<6} {mode:<8} {ver}")
    print()
    print("The apps take these addresses: transmitting end port 7010, receiving end port 7011.")
    return found


class Board:
    """SSH over password (BOARD_PW). Uploads via `cat >` so it works on both the E200's
    dropbear (no sftp) and the ADRV's openssh."""

    def __init__(self, host):
        import warnings
        warnings.filterwarnings("ignore")
        try:
            import paramiko
        except ImportError:
            sys.exit("need paramiko:  python -m pip install paramiko")
        pw = os.environ.get("BOARD_PW")
        if not pw:
            sys.exit("set BOARD_PW to the board's ssh password first")
        self.c = paramiko.SSHClient()
        self.c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.c.connect(host, username="root", password=pw, look_for_keys=False,
                       allow_agent=False, timeout=10)

    def sh(self, cmd, timeout=60):
        _, o, e = self.c.exec_command(cmd, timeout=timeout)
        return (o.read().decode(errors="replace") + e.read().decode(errors="replace")).strip()

    def put(self, data, remote, mode=None):
        _, o, e = self.c.exec_command(f"cat > '{remote}'", timeout=300)
        o.channel.sendall(data)
        o.channel.shutdown_write()
        rc = o.channel.recv_exit_status()
        if rc:
            sys.exit(f"upload {remote}: rc={rc} {e.read().decode(errors='replace')[:200]}")
        got = self.sh(f"md5sum '{remote}'").split()[0]
        if got != md5(data):
            sys.exit(f"upload {remote}: md5 mismatch")
        if mode:
            self.sh(f"chmod {mode} '{remote}'")

    def put_file(self, local, remote, mode=None):
        data = open(local, "rb").read()
        # A Windows clone (core.autocrlf) checks scripts and cfgs out with CRLF; the E200's
        # busybox sh then stops at the first `case`, and every cfg line grows a stray CR.
        # Text goes to the board with LF whatever the checkout did to it.
        if local.lower().endswith((".sh", ".cfg", ".py", ".txt", ".service", ".conf")):
            data = data.replace(b"\r\n", b"\n")
        self.put(data, remote, mode)

    def put_text(self, text, remote, mode=None):
        self.put(text.replace("\r\n", "\n").encode(), remote, mode)

    def close(self):
        self.c.close()


def ping(host):
    r = subprocess.run(["ping", "-n" if os.name == "nt" else "-c", "1",
                        "-w" if os.name == "nt" else "-W",
                        "1500" if os.name == "nt" else "1", host],
                       capture_output=True, text=True)
    return "TTL=" in r.stdout or "ttl=" in r.stdout


def wait_back(host, total=240):
    t0 = time.time()
    time.sleep(15)
    while time.time() - t0 < total:
        if ping(host):
            try:
                Board(host).close()
                return int(time.time() - t0)
            except Exception:
                pass
        time.sleep(3)
    return -1


def need(payload, board, files):
    d = os.path.join(payload, board)
    if not os.path.isdir(d):
        sys.exit(f"payload {d} not found — unpack the release archive, or pass --payload <dir>")
    miss = [f for f in files if not os.path.exists(os.path.join(d, f))]
    if miss:
        sys.exit(f"payload {d}: missing {', '.join(miss)}")
    return d


# ---------------------------------------------------------------- ADRV9364 (Kuiper) ----
ADRV_HOME = [("nyx-radio-node", "0755"), ("fir10MHz.ftr", None),
             ("nyxhop-start.py", "0755"), ("nyxctl.py", "0755")]


def flash_adrv(a, host, role):
    d = need(a.payload, "adrv9364",
             ["BOOT.BIN", "devicetree.dtb", "nyx-radio-node", "fir10MHz.ftr",
              "nyxhop-start.py", "nyxctl.py", "nyxhop.service", "nyx-radio-A.cfg", "nyx-radio-B.cfg"])
    if a.dry_run:
        print(f"[dry-run] ADRV9364 {host} role {role}")
        print(f"  /boot/BOOT.BIN, /boot/devicetree.dtb   (from {d})")
        print(f"  /home/root/: {', '.join(n for n, _ in ADRV_HOME)}, u-dma-buf.ko?, nyx-radio.cfg, nyx-role")
        print(f"  /etc/systemd/system/nyxhop.service; enable; reboot if boot changed")
        return
    b = Board(host)
    krn = b.sh("uname -r")
    print(f"== {host}: kernel {krn}")
    if b.sh("mount | grep -c ' /boot '") == "0":
        sys.exit("/boot (FAT) not mounted — not a Kuiper ADRV9364 board?")
    # boot image (backup the vendor originals once)
    changed = False
    for f in ("BOOT.BIN", "devicetree.dtb"):
        loc = os.path.join(d, f)
        want = md5(open(loc, "rb").read())
        if b.sh(f"md5sum /boot/{f} 2>/dev/null | cut -c1-32") == want:
            print(f"  /boot/{f}: already this build")
            continue
        b.sh(f"test -e /boot/{f}.nyx-orig || cp /boot/{f} /boot/{f}.nyx-orig 2>/dev/null; true")
        b.put_file(loc, f"/boot/{f}.new")
        b.sh(f"mv /boot/{f}.new /boot/{f} && sync")
        print(f"  /boot/{f}: written (original kept as {f}.nyx-orig)")
        changed = True
    # daemon + glue
    b.sh("systemctl stop nyxhop 2>/dev/null; killall nyx-radio-node 2>/dev/null; sleep 1; true")
    for name, mode in ADRV_HOME:
        b.put_file(os.path.join(d, name), f"/home/root/{name}", mode)
    if os.path.exists(os.path.join(d, "u-dma-buf.ko")):
        b.put_file(os.path.join(d, "u-dma-buf.ko"), "/home/root/u-dma-buf.ko")
    b.put_file(os.path.join(d, f"nyx-radio-{role}.cfg"), "/home/root/nyx-radio.cfg")
    # both roles' cfgs stay on the board: `role tx|rx` from an app switches between them
    for r in ("A", "B"):
        b.put_file(os.path.join(d, f"nyx-radio-{r}.cfg"), f"/home/root/nyx-radio-{r}.cfg")
    drop_old_role_mode(b, "/home/root", role)
    b.put_text(role + "\n", "/home/root/nyx-role")
    b.put_file(os.path.join(d, "nyxhop.service"), "/etc/systemd/system/nyxhop.service")
    b.put_text("options uio_pdrv_genirq of_id=generic-uio\n", "/etc/modprobe.d/nyxhop-uio.conf")
    # bootargs: the generic-uio driver must bind our PL nodes
    uenv = b.sh("cat /boot/uEnv.txt 2>/dev/null")
    lines = [l for l in uenv.splitlines() if l.strip()]
    ba = next((l for l in lines if l.startswith("bootargs=")), None)
    if ba and "uio_pdrv_genirq.of_id=generic-uio" not in ba:
        b.sh("test -e /boot/uEnv.txt.nyx-orig || cp /boot/uEnv.txt /boot/uEnv.txt.nyx-orig; true")
        ba2 = ba + " uio_pdrv_genirq.of_id=generic-uio"
        b.put_text("\n".join(ba2 if l is ba else l for l in lines) + "\n", "/boot/uEnv.txt")
        print("  /boot/uEnv.txt: added uio_pdrv_genirq.of_id=generic-uio")
    b.sh("systemctl daemon-reload && systemctl enable nyxhop >/dev/null 2>&1; true")
    print(f"  service nyxhop enabled (role {role})")
    if a.mac:
        b.put_text(f"[Match]\nOriginalName=eth0\n\n[Link]\nMACAddress={a.mac}\n",
                   "/etc/systemd/network/10-nyxhop-eth0.link")
    b.sh("sync")
    if changed:
        print(f"== reboot, waiting for {host} ...")
        b.sh("(sleep 1; systemctl reboot) >/dev/null 2>&1 &", timeout=5)
        b.close()
        t = wait_back(host)
        print(f"   back after {t} s" if t >= 0 else "   did NOT come back in 240 s — check a serial console")
    else:
        b.sh("systemctl restart nyxhop")
        b.close()
        print("== restarted nyxhop (no reboot needed)")
    if not a.no_verify:
        verify_adrv(host)


def verify_adrv(host):
    print("== verify", host)
    for _ in range(18):
        try:
            b = Board(host)
            break
        except Exception:
            time.sleep(5)
    else:
        print("  ssh not up"); return
    ver = ""
    for _ in range(12):
        ver = b.sh("python3 /home/root/nyxctl.py demod 2>&1 | grep -o 'ver=0x[0-9a-f]*' | head -1")
        if ver:
            break
        time.sleep(5)
    print("  fpga design:", ver or "no version (see /home/root/radio.out)")
    print("  service    :", b.sh("systemctl is-active nyxhop"))
    print("  uio        :", b.sh("for u in /sys/class/uio/uio*; do printf '%s=%s ' $(basename $u) $(cat $u/name); done"))
    print("  control    :", b.sh("python3 /home/root/nyxctl.py get 2>&1 | tr '\\n' ' ' | cut -c1-200"))
    b.close()


# ---------------------------------------------------------------------------- E200 ----
E200_SD = ["nyx.bit", "uEnv.txt", "uramdisk.image.gz"]


def flash_e200(a, host, role):
    d = need(a.payload, "e200", E200_SD + ["nyxhop", "nyx-radio-A-e200.cfg", "nyx-radio-B-e200.cfg"])
    nyxhop = os.path.join(d, "nyxhop")
    files = sorted(os.listdir(nyxhop))
    if a.dry_run:
        print(f"[dry-run] E200 {host} role {role}")
        print(f"  card root: {', '.join(E200_SD)}")
        print(f"  card/nyxhop/: {', '.join(files)} + nyx-radio.cfg (from nyx-radio-{role}-e200.cfg) + nyx-role")
        return
    b = Board(host)
    print(f"== {host}:", b.sh("uname -r"))
    if "mmcblk0p1" not in b.sh("cat /proc/partitions"):
        sys.exit("no /dev/mmcblk0p1 (SD card?)")
    b.sh("mkdir -p /mnt/sd; umount /mnt/sd 2>/dev/null; mount /dev/mmcblk0p1 /mnt/sd")
    if "/mnt/sd" not in b.sh("mount"):
        sys.exit("mount /dev/mmcblk0p1 failed: " + b.sh("mount /dev/mmcblk0p1 /mnt/sd 2>&1"))
    for f in E200_SD:
        b.put_file(os.path.join(d, f), "/mnt/sd/" + f)
        print(f"  /{f}")
    b.sh("mkdir -p /mnt/sd/nyxhop")
    for f in files:
        b.put_file(os.path.join(nyxhop, f), "/mnt/sd/nyxhop/" + f)
    b.put_file(os.path.join(d, f"nyx-radio-{role}-e200.cfg"), "/mnt/sd/nyxhop/nyx-radio.cfg")
    for r in ("A", "B"):  # both, so `role tx|rx` can switch without the tool
        b.put_file(os.path.join(d, f"nyx-radio-{r}-e200.cfg"), f"/mnt/sd/nyxhop/nyx-radio-{r}-e200.cfg")
    drop_old_role_mode(b, "/mnt/sd/nyxhop", role)
    b.put_text(role + "\n", "/mnt/sd/nyxhop/nyx-role")
    b.sh("rm -f /mnt/sd/nyxhop/ip")  # left by an earlier version of this tool
    print(f"  /nyxhop/: {len(files)} files + nyx-radio.cfg + nyx-role (role {role})")
    b.sh("sync; umount /mnt/sd")
    if a.no_verify:
        b.close()
        print("== done (no reboot); power-cycle then run --verify-only")
        return
    print("== reboot", host)
    b.sh("(sleep 1; reboot) >/dev/null 2>&1 &", timeout=5)
    b.close()
    t = wait_back(host)
    print(f"   back after {t} s" if t >= 0 else "   did NOT come back in 240 s")
    if t >= 0:
        verify_e200(host)


def verify_e200(host):
    print("== verify", host)
    b = Board(host)
    ver = ""
    for _ in range(20):
        ver = b.sh("/tmp/nyxhop/nyxctl --wait 2 demod 2>&1 | grep -o 'ver=0x[0-9a-f]*' | head -1")
        if ver:
            break
        time.sleep(3)
    print("  fpga design:", ver or "no version (see /tmp/nyxhop/radio.out)")
    print("  daemon     :", b.sh("ps | grep -v grep | grep nyx-radio-node | head -1")[:120])
    print("  control    :", b.sh("/tmp/nyxhop/nyxctl --wait 3 get 2>&1 | tr '\\n' ' ' | cut -c1-200"))
    print("  eth0       :", b.sh("ip -4 -o addr show eth0 | awk '{print $4}'"))
    b.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", help="board's current IP")
    ap.add_argument("--role", help="tx (transmitting) | rx (receiving); A/B also work")
    ap.add_argument("--scan", nargs="?", const="", metavar="NETWORK",
                    help="find boards on the network, e.g. --scan or --scan 192.168.1.0/24")
    ap.add_argument("--board", default="adrv9364", choices=["adrv9364", "e200"])
    ap.add_argument("--payload", default=os.path.join(os.getcwd(), "deploy"),
                    help="board images to flash (default ./deploy)")
    ap.add_argument("--mac", help="eth0 MAC (ADRV9364; the two boards must differ)")
    ap.add_argument("--verify-only", action="store_true")
    ap.add_argument("--no-verify", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, touch nothing")
    a = ap.parse_args()
    if a.scan is not None:
        nets = [a.scan.split("/")[0].rsplit(".", 1)[0]] if a.scan else local_nets()
        if not nets:
            sys.exit("could not work out this computer's network; pass one: --scan 192.168.1.0/24")
        scan(nets)
        return
    if not a.host or not a.role:
        sys.exit("--host and --role are required (or use --scan to find a board)")
    role = norm_role(a.role)
    if a.verify_only:
        (verify_adrv if a.board == "adrv9364" else verify_e200)(a.host)
        return
    (flash_adrv if a.board == "adrv9364" else flash_e200)(a, a.host, role)
    if not a.dry_run:
        print(f"\nDone: {a.host} is role {a.role} ({role}); it starts NyxHop on every power-up.")


if __name__ == "__main__":
    main()
