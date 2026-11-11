#!/bin/sh
# fc-status.sh - what runs on the board, one name=value a line. install.py reads it after an
# install and for its status command; over ssh it shows the same:  sh /home/root/px4/fc-status.sh
# The FC I/O block is only read when /boot/BOOT.BIN is the flight-controller image (fc-start.sh
# says why).
D=/home/root/px4
PATH=$D/bin:$PATH

B=$(md5sum /boot/BOOT.BIN | cut -d' ' -f1)
[ "$B" = "$(cat $D/fc-bootbin.md5 2>/dev/null)" ] && IMG=fc || IMG=other
echo "image=$IMG"
echo "bootbin=$B"
echo "mode=$(cat $D/mode 2>/dev/null)"
echo "kernel=$(uname -r)"
for s in nyxhop nyx-ipcam px4; do
	echo "$s=$(systemctl is-active $s 2>/dev/null)"
done

if [ $IMG = fc ]; then
	echo "fcio=$(busybox devmem 0x43C70000)"
	# the motor outputs' watchdog: 1 = nothing writes them (PX4 stopped), the FPGA holds them at stop
	echo "failsafe=$(( ($(busybox devmem 0x43C7005C) >> 31) & 1 ))"
	echo "pwm_outputs=$(busybox devmem 0x43C7007C)"
fi

# the ports the overlays brought up, each with its device (the FPGA's UARTs carry their port's name)
P=
[ -e /dev/spidev2.0 ] && P="imu:spidev2.0"
for t in /sys/class/tty/ttyS*; do
	[ -e $t/device/of_node/label ] && P="$P $(tr -d '\0' < $t/device/of_node/label):${t##*/}"
done
[ -e /dev/ttyPS1 ] && P="$P gps1:ttyPS1"
for c in /sys/class/net/can*; do
	[ -e $c ] && P="$P ${c##*/}"
done
echo "ports=${P# }"

# the radio: packets sent and control frames heard from the ground station, over two seconds
ctl() {
	python3 /home/root/nyxctl.py "$@" 2>/dev/null
}
sent() {
	ctl o64 | sed -n 's/^tx run=.* sent=\([0-9]*\).*/\1/p'
}
heard() {
	ctl stats | sed -n 's/^ota_rx=//p'
}
S1=$(sent); H1=$(heard)
sleep 2
S2=$(sent); H2=$(heard)
echo "radio_fpga=$(grep -a 'o64_rx running' /home/root/o64.out 2>/dev/null | tail -1 | sed -n 's/.*ver \(0x[0-9a-f]*\).*/\1/p')"
echo "radio_role=$(ctl role | sed -n 's/^role=//p')"
echo "radio_sent=$(( ${S2:-0} - ${S1:-0} ))"
echo "radio_heard=$(( ${H2:-0} - ${H1:-0} ))"
echo "licence=$(ctl license | sed -n 's/^lic_state=//p')"

if [ "$(systemctl is-active px4 2>/dev/null)" = active ]; then
	A=$(timeout 5 px4-listener sensor_accel 2>/dev/null)
	echo "imu_z=$(echo "$A" | awk '$1 == "z:" { print $2; exit }')"
	echo "imu_temp=$(echo "$A" | awk '$1 == "temperature:" { print $2; exit }')"
	echo "imu_errors=$(echo "$A" | awk '$1 == "error_count:" { print $2; exit }')"
fi
echo "px4_log=$(journalctl -u px4 -n 3 --no-pager -o cat 2>/dev/null | tr '\n' '|')"
