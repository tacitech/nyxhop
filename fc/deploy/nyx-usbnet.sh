#!/bin/sh
# The board's USB-C port plugged into a computer: the board shows up there as a network card, gives
# the computer an address and answers at 172.31.78.1 (ssh, install.py, nyx_flash.py). With a USB
# device plugged into the port instead (an Ethernet adapter, a camera) the port is a host as before.
#   nyx-usbnet.sh          set the USB network up and serve it (the service runs this)
#   nyx-usbnet.sh down     take it away
ME=172.31.78.1
PEER=172.31.78.2
G=/sys/kernel/config/usb_gadget/nyxhop
HERE=$(dirname "$0")

if [ "$1" = down ]; then
	[ -d $G ] || exit 0
	echo "" > $G/UDC 2>/dev/null
	rm -f $G/os_desc/c.1 $G/configs/c.1/rndis.usb0
	rmdir $G/configs/c.1/strings/0x409 $G/configs/c.1 $G/functions/rndis.usb0 $G/strings/0x409 $G 2>/dev/null
	exit 0
fi

UDC=$(ls /sys/class/udc 2>/dev/null | head -1)
if [ -z "$UDC" ]; then
	echo "nyx-usbnet: the USB port has no device side (it is a host only): nothing to do"
	exit 0
fi
if [ ! -d $G ]; then
	# addresses of the two ends from the board's own, so the computer sees the same card every time
	mac=$(cat /sys/class/net/eth0/address 2>/dev/null)
	[ -n "$mac" ] || mac=00:0a:35:00:01:22
	tail=${mac#*:}
	mkdir -p $G && cd $G || exit 1
	echo 0x1d6b > idVendor                      # Linux Foundation
	echo 0x0104 > idProduct                     # multifunction composite gadget
	echo 0x0100 > bcdDevice
	echo 0x0200 > bcdUSB
	echo 0xef > bDeviceClass                    # interface association: Windows takes RNDIS so
	echo 0x02 > bDeviceSubClass
	echo 0x01 > bDeviceProtocol
	mkdir -p strings/0x409
	echo NyxHop > strings/0x409/manufacturer
	echo "NyxHop board" > strings/0x409/product
	echo "$mac" | tr -d : > strings/0x409/serialnumber
	mkdir -p configs/c.1/strings/0x409
	echo network > configs/c.1/strings/0x409/configuration
	echo 0xc0 > configs/c.1/bmAttributes        # powered by the board, not by the bus
	echo 2 > configs/c.1/MaxPower
	mkdir -p functions/rndis.usb0
	echo "02:$tail" > functions/rndis.usb0/dev_addr
	echo "06:$tail" > functions/rndis.usb0/host_addr
	# Windows finds its own RNDIS driver by these
	echo 1 > os_desc/use
	echo 0xcd > os_desc/b_vendor_code
	echo MSFT100 > os_desc/qw_sign
	echo RNDIS > functions/rndis.usb0/os_desc/interface.rndis/compatible_id
	echo 5162001 > functions/rndis.usb0/os_desc/interface.rndis/sub_compatible_id
	ln -s functions/rndis.usb0 configs/c.1/
	ln -s configs/c.1 os_desc
	echo "$UDC" > UDC || exit 1
fi
ip addr replace $ME/24 dev usb0
ip link set usb0 up
exec python3 "$HERE/nyx-usbdhcp.py" usb0 $ME $PEER
