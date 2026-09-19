#!/bin/sh
# ipcam-loop.sh - keep nyx-ipcam running on the E200 (no systemd here). Started by local.sh.
# `save` on its console writes /tmp/nyxhop/nyx-ipcam.cfg and copies it to the SD card, which
# the boot overlay copies back into RAM at the next power-up.
D=/tmp/nyxhop
echo $$ > "$D/ipcam.pid"
SAVE="umount /mnt/sd 2>/dev/null; mkdir -p /mnt/sd && mount /dev/mmcblk0p1 /mnt/sd && mkdir -p /mnt/sd/nyxhop && cp '{path}' /mnt/sd/nyxhop/ && sync; umount /mnt/sd"
cd "$D" || exit 1
while true; do
  NYX_LOG_MAX_MB=4 "$D/nyx-ipcam" --config "$D/nyx-ipcam.cfg" --save-hook "$SAVE" >/dev/null 2>&1
  sleep 3
done
