#!/bin/sh
# start.sh - start the NyxHop radio daemon on the ANTSDR E200 stock Linux. The ramdisk
# overlay (S98nyxhop) copies /nyxhop from the SD card to /tmp/nyxhop and calls this file.
# It does what the Kuiper start-up script does on the other board:
#   1. read the role from nyx-role (tx|rx|A|B)
#   2. wait for the AD9361 phy, then set manual gain before the daemon starts
#   3. find the uio devices for the DMA and the capture buffer
#   4. start the daemon, logging to /tmp/nyxhop/radio.out
#   5. apply nyx-radio.cfg through the console on port 7202
D=/tmp/nyxhop
LOG=$D/radio.out
DA="--fir $D/fir10MHz.ftr --mod-base 0x43C30000 --demod-base 0x43C40000 --trig-base 0x43C00000 --video-hz 5745000000 --gpreg-base 0x41200000 --dac-base 0x79024000 --vcxo-base 0x43C70000 --state-dir /tmp/nyxhop --sd-dev /dev/mmcblk0p1 --lic-mtd /dev/mtd3:0x1df0000"
# The licence hour record lives in the last 64 KB sector of the QSPI Linux partition; it is
# named explicitly so the daemon may erase what is there. This FPGA design has no hardware
# AGC block, so no --agc-base is passed: the daemon uses its software AGC instead.

log() { echo "nyxhop: $*"; logger -t nyxhop "$*" 2>/dev/null; }

# send one console command, retrying until it is accepted (the daemon holds the radio busy
# for 10-20 s at start-up while it loads the FIR filter)
cfg() {
    j=0
    while [ $j -lt 8 ]; do
        "$D/nyxctl" --wait 3 "$1" >/dev/null 2>&1 && return 0
        sleep 3
        j=$((j + 1))
    done
    log "cfg FAIL: $1"
    return 1
}

nyx_start() {
    ROLE=$(cat "$D/nyx-role" 2>/dev/null | tr 'A-Z' 'a-z')
    case "$ROLE" in
        tx|a) ROLE=A; RXLO=5835000000; TXLO=5745000000; GAIN=62 ;;
        rx|b) ROLE=B; RXLO=5745000000; TXLO=5835000000; GAIN=60 ;;
        *) log "nyx-role '$ROLE' is not tx/rx - stopping"; return ;;
    esac
    log "role=$ROLE - starting"
    P=""
    i=0
    while [ $i -lt 40 ]; do
        for d in /sys/bus/iio/devices/iio:device*; do
            [ -e "$d/in_voltage_rf_bandwidth" ] && P="$d"
        done
        [ -n "$P" ] && break
        sleep 1
        i=$((i + 1))
    done
    [ -n "$P" ] || { log "AD9361 phy not found"; return; }
    # Board reference error in parts per billion (the E200 has no disciplined TCXO):
    # positive means the board LO sits ABOVE nominal, and the daemon subtracts it from
    # EVERY LO request. Do NOT use the xo_correction sysfs knob: an odd XO makes the
    # BBPLL 983039991 instead of 983040000, the divider chain rounds off by 1 Hz, and
    # the ad9361 driver compares those clocks with EXACT equality in
    # ad9361_tx_quad_calib (clkrf == 2*clktf). It then reports "Unhandled case", leaves
    # TX quadrature uncalibrated and returns EINVAL for every LO change.
    PPB=$(cat "$D/nyx-loppb" 2>/dev/null | tr -d " 
")
    if [ -n "$PPB" ]; then
        DA="$DA --lo-ppb $PPB"
        log "lo-ppb=$PPB"
    fi
    echo manual  > "$P/in_voltage0_gain_control_mode"
    echo "$GAIN" > "$P/in_voltage0_hardwaregain"
    # The LOs are set through the daemon (cfg rxfreq/txfreq) so lo-ppb is applied;
    # writing sysfs directly here would skip the correction and tune the driver twice.
    UIO=""
    CAP=""
    for u in /sys/class/uio/uio*; do
        n=$(cat "$u/name" 2>/dev/null)
        [ "$n" = "dma" ] && UIO=$(basename "$u")
        [ "$n" = "capbuf" ] && CAP=$(basename "$u")
    done
    [ -n "$UIO" ] || { log "uio 'dma' not found (did the boot script patch the device tree?)"; return; }
    [ -n "$CAP" ] || { log "uio 'capbuf' not found (did the boot script patch the device tree?)"; return; }
    killall nyx-radio-node 2>/dev/null
    sleep 1
    cd "$D" || return
    eval "nohup $D/nyx-radio-node --uio $UIO --udmabuf uio:$CAP $DA > $LOG 2>&1 &"
    log "daemon: --uio $UIO --udmabuf uio:$CAP $DA"
    if [ -f "$D/nyx-radio.cfg" ]; then
        sed -e 's/#.*//' "$D/nyx-radio.cfg" | while read -r line; do
            [ -n "$line" ] && cfg "$line"
        done
    else
        log "no nyx-radio.cfg - falling back to minimal LO/gain"
        cfg "rxfreq $RXLO"
        cfg "txfreq $TXLO"
        cfg "txgain 0"
        cfg "agc manual"
        cfg "rxgain $GAIN"
    fi
    log "$ROLE READY"
}

case "$1" in
    start) nyx_start & ;;
    stop) killall nyx-radio-node 2>/dev/null ;;
    restart) killall nyx-radio-node 2>/dev/null; sleep 1; nyx_start & ;;
    *) echo "usage: $0 {start|stop|restart}"; exit 1 ;;
esac
exit 0
