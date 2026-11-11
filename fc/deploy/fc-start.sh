#!/bin/sh
# fc-start.sh - run by px4.service before PX4 starts. It checks that the FPGA image is the one with
# the flight-controller I/O, turns the board's ports on and points the IMU's SPI bus, by the mode
# in /home/root/px4/mode (PX4 then runs nyxhop-<mode>.config):
#   bb   the NyxHop board: the IMU on it, CH1..8, GPS1 and GPS2, RC, TELEM1, the I2C ports, CAN,
#        the battery ADC
#   hil  the IMU emulated in the FPGA for simulation in the loop (PX4's SIH writes accel and gyro)
#   fc   an ICM-42688-P wired to the radio module's breakout header
# The ports come up as device tree overlays, compiled ahead (the *.dtbo beside this script) and
# applied once a boot. The FC I/O block is only read when /boot/BOOT.BIN is the image the installer
# wrote (its md5 is in fc-bootbin.md5): another FPGA image has nothing at that address, and a read
# there hangs the board.
D=/home/root/px4
MODE=$(cat $D/mode 2>/dev/null || echo hil)
OV=/sys/kernel/config/device-tree/overlays
DRV=/sys/bus/platform/drivers

die() {
	echo "fc-start: $*: PX4 not started"
	exit 1
}

overlay() {	# <name> <file>
	[ "$(cat $OV/$1/status 2>/dev/null)" = applied ] && return 0
	[ -f $D/$2 ] || die "$2 is missing"
	rmdir $OV/$1 2>/dev/null
	mkdir $OV/$1 && cat $D/$2 > $OV/$1/dtbo && [ "$(cat $OV/$1/status)" = applied ] && return 0
	rmdir $OV/$1 2>/dev/null
	die "the overlay $2 was refused"
}

xadc_has_vpvn() {	# the VP/VN input is the XADC's voltage8
	for x in /sys/bus/iio/devices/iio:device*; do
		[ "$(cat $x/name 2>/dev/null)" = xadc ] && ls $x | grep -q '^in_voltage8_.*raw$' && return 0
	done
	return 1
}

[ "$(md5sum /boot/BOOT.BIN | cut -d' ' -f1)" = "$(cat $D/fc-bootbin.md5 2>/dev/null)" ] ||
	die "/boot/BOOT.BIN is not the flight-controller image"
V=$(busybox devmem 0x43C70000)
[ "$((V >> 16))" = "$((0x4643))" ] || die "no FC I/O block in the FPGA ($V)"
[ -d $OV ] || mount -t configfs none /sys/kernel/config 2>/dev/null
[ -d $OV ] || die "this kernel takes no device tree overlays"

# SPI1 and the IMU on it: /dev/spidev2.0
overlay fcio fcio-spi.dtbo

if [ "$MODE" = bb ]; then
	[ "$((V & 0xFFFF))" -ge 3 ] || die "the FPGA image is not the NyxHop board's ($V)"
	[ -d $DRV/of_serial ] || die "this kernel has no 16550 UART driver (/boot/uImage is not the image's)"
	# RC, TELEM1 and GPS2 (UARTs in the FPGA: /dev/ttyS0..2), the sensors' I2C bus
	overlay nyxbb nyxbb-io.dtbo
	# GPS1 (/dev/ttyPS1), the I2C of the GPS1 port, the supply voltage on the XADC
	overlay nyxbbps nyxbb-ps.dtbo
	[ -d $DRV/xilinx_can ] && overlay nyxbbcan nyxbb-can.dtbo
	# the XADC driver reads its channels from the device tree when it binds: bound again
	if ! xadc_has_vpvn; then
		A=f8007100.adc
		X=$(basename "$(readlink -f /sys/bus/platform/devices/$A/driver)")
		if [ -d $DRV/$X ]; then
			echo $A > $DRV/$X/unbind
			sleep 0.5
			echo $A > $DRV/$X/bind
		fi
	fi
fi

n=0
while [ ! -e /dev/spidev2.0 ] && [ $n -lt 10 ]; do
	sleep 0.5
	n=$((n + 1))
done
[ -e /dev/spidev2.0 ] || die "no /dev/spidev2.0 (overlay: $(cat $OV/fcio/status 2>/dev/null))"

# the IMU's bus: 0x1 the emulated chip, 0x2 the real one
case $MODE in
	fc|bb) busybox devmem 0x43C70004 32 0x2 ;;
	*)     busybox devmem 0x43C70004 32 0x1 ;;
esac
echo "fc-start: mode $MODE, FC I/O $V, CTRL $(busybox devmem 0x43C70004)"
