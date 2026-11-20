#!/bin/sh
# NyxHop for the PlutoSky R2: copies pluto.frm onto the board's PlutoSDR drive, ejects it and
# waits for the board to come back with the new firmware. Linux (lsblk/udisks or eject) and Mac.
here=$(cd "$(dirname "$0")" && pwd)
frm="$here/pluto.frm"
[ -f "$frm" ] || { echo "pluto.frm is not next to this script."; exit 1; }

mac=0
[ "$(uname)" = "Darwin" ] && mac=1

# the drive: its device on Linux, its mount point on a Mac; empty when it is not there
find_drive() {
    if [ $mac = 1 ]; then
        [ -d /Volumes/PlutoSDR ] && echo /Volumes/PlutoSDR
    else
        lsblk -rno NAME,LABEL 2>/dev/null | awk '$2 == "PlutoSDR" { print "/dev/" $1; exit }'
    fi
}
wait_drive() {   # wait_drive yes|no seconds
    i=0
    while [ $i -lt "$2" ]; do
        d=$(find_drive)
        if [ "$1" = yes ] && [ -n "$d" ]; then echo "$d"; return 0; fi
        if [ "$1" = no ] && [ -z "$d" ]; then return 0; fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

echo "Looking for the PlutoSDR drive..."
dev=$(wait_drive yes 60) || {
    echo "No PlutoSDR drive after a minute."
    echo "Is the board's USB port plugged in (not the DEBUG port in the middle)? Does it have enough power? Try both Type-C ports."
    exit 1
}
if [ $mac = 1 ]; then
    mp=$dev
else
    mp=$(lsblk -rno MOUNTPOINT "$dev" 2>/dev/null | head -1)
    if [ -z "$mp" ] && command -v udisksctl >/dev/null 2>&1; then
        udisksctl mount -b "$dev" >/dev/null 2>&1
        mp=$(lsblk -rno MOUNTPOINT "$dev" 2>/dev/null | head -1)
    fi
    if [ -z "$mp" ]; then
        mp=$(mktemp -d)
        sudo mount "$dev" "$mp" || { echo "Could not mount $dev."; exit 1; }
    fi
fi
# a PlutoSDR shows the same drive: its info.html names a Zynq-7010, the R2's a Zynq-7020
if [ -f "$mp/info.html" ] && ! grep -q Z7020 "$mp/info.html"; then
    echo "The board on $dev is not a PlutoSky R2 (its model is not a Zynq-7020). This firmware is for the R2 only: nothing was copied."
    exit 1
fi
echo "Found it ($dev). Copying the firmware..."
cp "$frm" "$mp/pluto.frm" && sync
a=$(md5sum "$frm" 2>/dev/null | cut -d' ' -f1); [ -n "$a" ] || a=$(md5 -q "$frm")
b=$(md5sum "$mp/pluto.frm" 2>/dev/null | cut -d' ' -f1); [ -n "$b" ] || b=$(md5 -q "$mp/pluto.frm")
[ "$a" = "$b" ] || { echo "The copy came out different. Run this again."; exit 1; }

echo "Copied. Ejecting the drive - the board writes the firmware now. Don't unplug it."
if [ $mac = 1 ]; then
    diskutil eject /Volumes/PlutoSDR >/dev/null
else
    if command -v udisksctl >/dev/null 2>&1; then
        udisksctl unmount -b "$dev" >/dev/null 2>&1 || sudo umount "$mp"
    else
        sudo umount "$mp"
    fi
    eject "$dev" 2>/dev/null || sudo eject "$dev"
fi
wait_drive no 120 >/dev/null

echo "Waiting for the board to restart (one to three minutes)..."
wait_drive yes 360 >/dev/null || {
    echo "The drive did not come back in six minutes."
    echo "Unplug the board and plug it in again. If a file named FAILED... is on the drive, the copy was damaged: run this again."
    echo "If there is no drive at all, see 'Install guide.pdf', section 8."
    exit 1
}
echo "The board is back. NyxHop restarts it once more the first time - a minute or so."
if wait_drive no 90 >/dev/null; then
    wait_drive yes 240 >/dev/null
fi
echo
echo "Done. The firmware is on the board."
echo "Open the NyxHop app, choose the role for this end and enter the board's address (192.168.1.10 over Ethernet)."
