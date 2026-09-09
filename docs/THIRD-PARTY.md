# Third-party components

The board images under `deploy/` are not built from nothing: they wrap the board vendor's boot
loader and Linux userland around the NyxHop FPGA design and radio daemon. Those parts keep their
own licences, which are not the MIT licence covering this repository's source and not the EULA
covering the NyxHop binaries.

| component | where it is | licence |
|---|---|---|
| U-Boot 2018.01 (Analog Devices build) and the Xilinx first-stage boot loader | inside `deploy/adrv9364/BOOT.BIN` | GPL-2.0 (U-Boot); Xilinx terms (FSBL) |
| Linux device tree derived from Analog Devices' tree | `deploy/adrv9364/devicetree.dtb` | GPL-2.0 / X11, as in the Linux tree |
| `u-dma-buf` kernel module by ikwzm | `deploy/adrv9364/u-dma-buf.ko` | Dual BSD/GPL |
| Buildroot root filesystem with BusyBox 1.31.1, from the ANTSDR stock firmware | inside `deploy/e200/uramdisk.image.gz` | GPL-2.0 and the licences of the packages it contains |

## Getting the source of the GPL parts

For the components above that are under the GPL, you are entitled to the corresponding source.
Take it from where we did:

* U-Boot and the Linux device tree for the ADRV9364: Analog Devices' repositories,
  <https://github.com/analogdevicesinc/u-boot-xlnx> and
  <https://github.com/analogdevicesinc/linux>, at the tag matching the version string in the
  binary (`strings BOOT.BIN | grep U-Boot` prints it).
* `u-dma-buf`: <https://github.com/ikwzm/udmabuf>.
* The E200 root filesystem: the ANTSDR firmware sources from MicroPhase.

If you would rather have it from us, ask and we will send the exact sources those binaries were
built from, on a medium of your choice, for no more than the cost of the copy.

## In the applications

The apps link ordinary Rust crates from crates.io, each under its own permissive licence, plus
**openh264** (Cisco, BSD-2-Clause) for H.264 encoding and decoding. `cargo tree` in this
repository lists them all with versions. Note that H.264 is a patented format: the openh264
source is free, but whether you owe patent royalties for shipping an H.264 encoder in your own
product is a question for you and the patent pools, not something this licence answers.
