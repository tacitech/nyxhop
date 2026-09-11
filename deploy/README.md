# Board images (flash these)

Prebuilt, per board type. The flashing tool `../link/scripts/nyx_flash.py` writes one of these
onto a board over SSH and verifies it: it does not build anything. These are the product
binaries (FPGA design, radio daemon, boot files); their source is not in this repository.

**Licensing.** Everything in this folder is covered by the EULA (`../docs/EULA.md`), not by the
NyxHop Source Licence that covers the source code in this repository. The board images also contain
third-party components (U-Boot, BusyBox and others) under their own licences, listed with their
source locations in `../docs/THIRD-PARTY.md`.

    deploy/adrv9364/   BOOT.BIN  devicetree.dtb  nyx-radio-node  nyxhop-start.py  nyxctl.py
                       nyxhop.service  u-dma-buf.ko  fir10MHz.ftr  nyx-radio-A.cfg  nyx-radio-B.cfg
    deploy/e200/       nyx.bit  uEnv.txt  uramdisk.image.gz  nyxhop/  nyx-radio-A-e200.cfg  nyx-radio-B-e200.cfg

Usage (from the repository root):

    set BOARD_PW=analog                  # PowerShell: $env:BOARD_PW = "analog"
    python link/scripts/nyx_flash.py --host <ip> --role tx            # ADRV9364 aircraft
    python link/scripts/nyx_flash.py --host <ip> --role rx            # ADRV9364 ground
    python link/scripts/nyx_flash.py --host 192.168.0.12 --role rx --board e200
