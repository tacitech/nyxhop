#!/usr/bin/env python3
"""nyxctl - send ONE console command to the radio daemon (127.0.0.1:7202) and print the
reply. For use on the board, where there is no netcat:  python3 nyxctl.py trig
"""
import socket
import sys
import time


def main():
    line = " ".join(sys.argv[1:]) or "get"
    try:
        s = socket.create_connection(("127.0.0.1", 7202), timeout=3)
    except OSError as e:
        print(f"err connect: {e}")
        sys.exit(1)
    s.sendall((line + "\n").encode())
    s.settimeout(2.0)
    d = b""
    t0 = time.time()
    while time.time() - t0 < 2.0:
        try:
            c = s.recv(8192)
        except OSError:
            break
        if not c:
            break
        d += c
        if d.rstrip().endswith(b"ok") or b"\nerr" in d or d.startswith(b"err"):
            break
    s.close()
    print(d.decode(errors="replace").strip())


if __name__ == "__main__":
    main()
