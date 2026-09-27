#!/usr/bin/env python3
"""nyx_pluto - NyxHop onto the PlutoSDRs plugged into this computer, through their own USB drives.

    python link/scripts/nyx_pluto.py              # every Pluto here: NyxHop, an address of its own, checked
    python link/scripts/nyx_pluto.py --pair       # two Plutos here: also one ground, one aircraft, paired
    python link/scripts/nyx_pluto.py --list       # show them and what would change; write nothing
    python link/scripts/nyx_pluto.py --net-only   # only the addresses, no firmware
    python link/scripts/nyx_pluto.py --frm <file> # another firmware, e.g. ADI's pluto.frm to go back to stock

It does what the README's manual steps do, for every Pluto whose drive it finds:
  1. reads the Pluto's serial and address from its drive (info.html, config.txt)
  2. gives it an address of its own when it needs one - two Plutos on one computer, or another
     network card of this computer (a virtual machine's host-only adapter, say) on the same
     subnet: 192.168.N.1 for the Pluto and 192.168.N.10 for this computer, written into
     config.txt
  3. copies pluto.frm onto the drive, asks for a restart in config.txt and ejects the drive; the
     Pluto writes the firmware and the address, then restarts (a minute or two, LED blinking
     fast - do not unplug it then)
  4. waits for NyxHop to answer on the Pluto's address and prints what the apps need
Only Python 3 is needed. The Pluto's own update mechanism does the writing, and an update keeps
role, pairing and channel tables.
"""
import argparse
import glob
import hashlib
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
FRM = os.path.join(ROOT, "deploy", "pluto", "pluto.frm")
CONSOLE_PORT = 7202


# ------------------------------------------------------------------ the drives ----
def drive_roots():
    """Folders where a Pluto's USB drive may be mounted."""
    if sys.platform == "win32":
        return [f"{c}:\\" for c in "DEFGHIJKLMNOPQRSTUVWXYZ"]
    if sys.platform == "darwin":
        return glob.glob("/Volumes/*/")
    return glob.glob("/media/*/*/") + glob.glob("/run/media/*/*/") + glob.glob("/media/*/") + glob.glob("/mnt/*/")


def read(path):
    with open(path, "rb") as f:
        return f.read().decode("utf-8", errors="replace")


def info_field(html, name):
    m = re.search(rf"<td>{name}</td>\s*<td>([^<]*)</td>", html)
    return m.group(1).strip() if m else ""


def ini_get(text, section, key):
    sec = None
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("[") and s.endswith("]"):
            sec = s[1:-1]
        elif sec == section and "=" in s and not s.startswith("#"):
            k, v = s.split("=", 1)
            if k.strip() == key:
                return v.strip()
    return ""


def ini_set(text, section, key, value):
    """config.txt with `key = value` in [section], the rest of the file as it was."""
    lines = text.splitlines()
    sec, start, end = None, None, len(lines)
    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith("[") and s.endswith("]"):
            if sec == section:
                end = i
                break
            sec = s[1:-1]
            if sec == section:
                start = i
    if start is None:
        lines += ["", f"[{section}]", f"{key} = {value}"]
    else:
        for i in range(start + 1, end):
            s = lines[i].strip()
            if "=" in s and not s.startswith("#") and s.split("=", 1)[0].strip() == key:
                lines[i] = f"{key} = {value}"
                break
        else:
            lines.insert(end, f"{key} = {value}")
    nl = "\r\n" if "\r\n" in text else "\n"
    return nl.join(lines) + nl


def find_plutos():
    found = []
    for root in drive_roots():
        cfg, info = os.path.join(root, "config.txt"), os.path.join(root, "info.html")
        try:
            if not (os.path.isfile(cfg) and os.path.isfile(info)):
                continue
            html, text = read(info), read(cfg)
        except OSError:
            continue
        if "PlutoSDR" not in html:
            continue
        found.append({
            "drive": root,
            "serial": info_field(html, "Serial"),
            "model": info_field(html, "Model"),
            "build": info_field(html, "Build"),
            "ip": ini_get(text, "NETWORK", "ipaddr"),
            "host": ini_get(text, "NETWORK", "ipaddr_host"),
            "cfg": text,
        })
    return sorted(found, key=lambda p: p["serial"])


# ----------------------------------------------------------------- addresses ----
def local_ipv4():
    """Every IPv4 address of this computer."""
    ips = set()
    try:
        ips |= {a[4][0] for a in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)}
    except OSError:
        pass
    for cmd in (["ip", "-4", "-o", "addr", "show"], ["ifconfig"]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        ips |= set(re.findall(r"inet (?:addr:)?(\d+\.\d+\.\d+\.\d+)", out))
    return {ip for ip in ips if not ip.startswith("127.")}


def net24(ip):
    return ".".join(ip.split(".")[:3])


def plan(plutos, ips):
    """(pluto, new ip, new host, why) for each Pluto whose address must change."""
    hosts = {p["host"] for p in plutos}
    # this computer's other networks (a Pluto's own link shows here as its ipaddr_host)
    used = {net24(ip): f"this computer's {ip}" for ip in ips if ip not in hosts}
    changes = []
    for p in plutos:
        n = net24(p["ip"]) if p["ip"] else ""
        if not n or net24(p["host"] or "0.0.0.0") != n or p["ip"] == p["host"]:
            why = "its address and this computer's are not one subnet"
        elif n in used:
            why = f"{n}.x is taken by {used[n]}"
        else:
            used[n] = f"the Pluto at {p['ip']}"
            continue
        k = next(k for k in range(2, 255) if f"192.168.{k}" not in used)
        used[f"192.168.{k}"] = f"the Pluto at 192.168.{k}.1"
        changes.append((p, f"192.168.{k}.1", f"192.168.{k}.10", why))
    return changes


# ------------------------------------------------------------------ firmware ----
def check_frm(path):
    """A Pluto firmware file whole: the image, then its md5 in hex and a newline."""
    data = open(path, "rb").read()
    if len(data) < 1 << 20 or data[-1:] != b"\n":
        sys.exit(f"{path}: not a Pluto firmware file ({len(data)} bytes)")
    if hashlib.md5(data[:-33]).hexdigest() != data[-33:-1].decode(errors="replace"):
        sys.exit(f"{path}: its checksum does not match - download it again")
    return data


def write_file(path, data):
    with open(path, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())


def eject_once(root):
    """True when the eject went out (on Linux the eject command says so; elsewhere the check is
    the drive going away, see eject)."""
    if sys.platform == "win32":
        letter = root[:2]
        ps = f"(New-Object -ComObject Shell.Application).Namespace(17).ParseName('{letter}').InvokeVerb('Eject')"
        subprocess.run(["powershell", "-NoProfile", "-Command", ps], check=False)
        return True
    if sys.platform == "darwin":
        return subprocess.run(["diskutil", "eject", root]).returncode == 0
    part = subprocess.run(["findmnt", "-n", "-o", "SOURCE", "--target", root],
                          capture_output=True, text=True).stdout.strip()
    subprocess.run(["sync"], check=False)
    if subprocess.run(["udisksctl", "unmount", "-b", part], capture_output=True).returncode != 0:
        subprocess.run(["umount", root], check=False)
    disk = re.sub(r"p?\d+$", "", part) if re.search(r"\d$", part) else part
    return subprocess.run(["eject", disk]).returncode == 0


def eject(root):
    """Eject the drive: that is what makes the Pluto act on what was written to it. Windows can
    ignore an eject without a word while the files just written are still held (seen 27/9), so
    the drive has to go away, else it is tried again."""
    time.sleep(1)
    for _ in range(3):
        sent = eject_once(root)
        if sys.platform.startswith("linux"):
            if sent:
                return True
            continue
        t0 = time.time()
        while time.time() - t0 < 10:
            if not os.path.exists(os.path.join(root, "config.txt")):
                return True
            time.sleep(0.2)
    print(f"  {root} did not eject: eject it yourself (Explorer, Finder or the file manager) - the "
          f"Pluto acts on the eject")
    return False


# ------------------------------------------------------------------- console ----
def console(ip, cmd, timeout=3.0):
    """One command to the NyxHop console on the Pluto, and its reply ("" when nothing answers)."""
    try:
        s = socket.create_connection((ip, CONSOLE_PORT), timeout=timeout)
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
            if re.search(rb"(^|\n)(ok|err[^\n]*)\s*$", buf):
                break
        return buf.decode(errors="replace")
    finally:
        s.close()


def kv(reply, key):
    m = re.search(rf"{key}=(\S+)", reply)
    return m.group(1) if m else ""


def set_role(ip, role):
    if kv(console(ip, "role"), "role") == role:
        return True
    console(ip, f"role {role}")
    time.sleep(3)
    t0 = time.time()
    while time.time() - t0 < 60:
        if kv(console(ip, "role"), "role") == role:
            return True
        time.sleep(2)
    return False


def same_tables(ground, aircraft):
    """The two ends meet only on the same tables (the pairing frame goes out on the first
    control channel): give the aircraft end the ground end's, when they differ."""
    g, a = console(ground, "hop status"), console(aircraft, "hop status")
    moved = False
    for key, cmd in (("hop_table", "hop chans"), ("hop_ctl_table", "hop ctlchans")):
        want = kv(g, key)
        if want and kv(a, key) != want:
            print(f"  aircraft end {key.replace('hop_', '').replace('_', ' ')} {kv(a, key)} -> {want} (the ground end's)")
            console(aircraft, f"{cmd} {want}", timeout=6)
            moved = True
    if moved:
        time.sleep(10)          # a control table changes at the next epoch


def pair(ground, aircraft):
    """The ground end binds, the aircraft end accepts: the key goes over the air."""
    same_tables(ground, aircraft)
    for attempt in range(3):
        console(aircraft, "link accept 60")
        tag = kv(console(ground, "link bind 20"), "link_tag")
        t0 = time.time()
        while time.time() - t0 < 25:
            if kv(console(aircraft, "link"), "link_state") == f"bound:{tag}":
                return tag
            time.sleep(2)
        print(f"  pairing attempt {attempt + 1}: the aircraft end did not hear it, again")
    return ""


# ---------------------------------------------------------------------- main ----
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="show the Plutos and the plan, write nothing")
    ap.add_argument("--net-only", action="store_true", help="only the addresses, no firmware")
    ap.add_argument("--frm", default=FRM, help="firmware file (default: deploy/pluto/pluto.frm)")
    ap.add_argument("--pair", action="store_true", help="two Plutos: first ground, second aircraft, paired")
    ap.add_argument("--yes", action="store_true", help="do not ask before writing")
    a = ap.parse_args()

    plutos = find_plutos()
    if not plutos:
        sys.exit("No PlutoSDR drive found. Plug the Pluto in and wait until its drive shows up "
                 "(a PlutoSDR drive with config.txt and info.html), then run this again.")
    changes = {id(p): (ip, host, why) for p, ip, host, why in plan(plutos, local_ipv4())}
    print(f"{'drive':<14}{'serial':<14}{'firmware':<10}{'address':<16}{'this computer':<16}")
    for p in plutos:
        print(f"{p['drive']:<14}{p['serial'][:12]:<14}{p['build']:<10}{p['ip']:<16}{p['host']:<16}")
        if id(p) in changes:
            ip, host, why = changes[id(p)]
            print(f"{'':<14}-> new address {ip} (this computer {host}): {why}")
    if a.pair and len(plutos) != 2:
        sys.exit(f"--pair needs exactly two Plutos here, found {len(plutos)}")
    frm = None if a.net_only else check_frm(a.frm)
    todo = []
    if frm:
        todo.append(f"write {os.path.basename(a.frm)} to {len(plutos)} Pluto(s)")
    if changes:
        todo.append(f"change {len(changes)} address(es)")
    if a.pair:
        todo.append("pair the two")
    if a.list:
        print("Would: " + (", ".join(todo) or "nothing") + " (--list: nothing written)")
        return
    if not todo:
        print("Nothing to change.")
        return
    if not a.yes and input("Go ahead: " + ", ".join(todo) + "? [y/N] ").strip().lower() != "y":
        return

    for p in plutos:
        # NyxHop already running there answers until the Pluto has written its flash and
        # restarts: only an answer after it went away counts
        p["restarts"] = bool(frm) or id(p) in changes
        p["seen_down"] = not console(p["ip"], "role")
        text = p["cfg"]
        if id(p) in changes:
            ip, host, _ = changes[id(p)]
            text = ini_set(text, "NETWORK", "ipaddr", ip)
            text = ini_set(text, "NETWORK", "ipaddr_host", host)
            text = ini_set(text, "NETWORK", "netmask", "255.255.255.0")
            text = ini_set(text, "ACTIONS", "reset", "1")      # restart with the new address
            p["ip"], p["host"] = ip, host
            write_file(os.path.join(p["drive"], "config.txt"), text.encode())
        if frm:
            print(f"{p['drive']}: copying the firmware ...")
            write_file(os.path.join(p["drive"], "pluto.frm"), frm)
        if frm or id(p) in changes:
            print(f"{p['drive']}: ejecting - the Pluto writes it and restarts, do not unplug it")
            eject(p["drive"])

    print("Waiting for NyxHop to answer (a minute or two) ...")
    t0 = time.time()
    left = [p for p in plutos]
    while left and time.time() - t0 < 480:
        for p in list(left):
            role = kv(console(p["ip"], "role"), "role")
            if not role:
                p["seen_down"] = True
            elif p["seen_down"] or not p["restarts"]:
                p["role"] = role
                print(f"  {p['ip']:<16}NyxHop running, role {role}")
                left.remove(p)
        time.sleep(2)
    for p in left:
        print(f"  {p['ip']:<16}no answer from NyxHop" if p["seen_down"] else
              f"  {p['ip']:<16}did not restart: was its drive ejected?")
    if a.pair and all(p.get("role") for p in plutos):
        ground, aircraft = plutos
        print(f"Pairing: {ground['ip']} ground, {aircraft['ip']} aircraft ...")
        if not (set_role(ground["ip"], "rx") and set_role(aircraft["ip"], "tx")):
            sys.exit("  a Pluto did not take its role")
        ground["role"], aircraft["role"] = "rx", "tx"
        tag = pair(ground["ip"], aircraft["ip"])
        print(f"  paired, key tag {tag}" if tag else "  not paired: press Link aircraft in the ground app")

    print()
    for p in plutos:
        if not p.get("role"):
            print(f"{p['ip']}: no answer. Is this computer's address on its USB network "
                  f"{p['host']}? Unplug the Pluto, plug it in again, and run this with --list.")
            continue
        mode = "rx" if p["role"] == "rx" else "tx"
        print(f"{p['ip']}: {'ground' if mode == 'rx' else 'aircraft'} end -> nyxhop --mode {mode} --board {p['ip']}")


if __name__ == "__main__":
    main()
