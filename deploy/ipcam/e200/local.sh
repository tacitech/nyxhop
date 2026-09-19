#!/bin/sh
# local.sh - extra start-up steps of this E200 card, run by start.sh at boot (start) and
# on stop. Here: the IP camera app (nyx-ipcam). It runs whatever the board's role is and
# follows the role by itself, so a role change (daemon restart) leaves it running.
D=/tmp/nyxhop
case "$1" in
  start)
    [ -f "$D/nyx-ipcam" ] || exit 0
    if [ -f "$D/ipcam.pid" ] && kill -0 "$(cat "$D/ipcam.pid")" 2>/dev/null; then
      exit 0
    fi
    chmod +x "$D/nyx-ipcam"
    setsid sh "$D/ipcam-loop.sh" >/dev/null 2>&1 < /dev/null &
    ;;
  stop)
    [ -f "$D/ipcam.pid" ] && kill "$(cat "$D/ipcam.pid")" 2>/dev/null
    rm -f "$D/ipcam.pid"
    killall nyx-ipcam 2>/dev/null
    ;;
esac
exit 0
