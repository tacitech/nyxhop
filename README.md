# NyxHop

**A frequency-hopping OFDM radio link for drones and robots, on off-the-shelf SDR boards.**

![source available](https://img.shields.io/badge/source-source--available-blue)
![board images EULA](https://img.shields.io/badge/board%20images-EULA-lightgrey)
![Rust](https://img.shields.io/badge/rust-1.85%2B-orange)
![platforms](https://img.shields.io/badge/platforms-Windows%20%7C%20Linux%20%7C%20Android-lightgrey)
![bands](https://img.shields.io/badge/70%20MHz-6%20GHz-green)

NyxHop turns a pair of software-defined-radio boards, an ADRV9364-Z7020 or an ANTSDR E200, into
a private frequency-hopping data link, and gives you the software around it. The link moves
blocks of bytes; what you put in them is your business: video, telemetry, files, your own
protocol, all at once if you like. Anywhere between 70 MHz and 6 GHz that your hardware and
your local rules allow.

On the bench it carries an H.264 camera stream at **30 fps**, **28 ms** from camera to screen (61 ms with the two-layer simulcast switched on, which keeps a picture through fades).

![The ground app: video on the left, the channel tables on the right](docs/img/ground-app.png)

*The ground app holding 5795 MHz out of its video table, with the control channel on its own
table down at 2.4 GHz.*

If you know the OcuSync-style digital links on commercial drones, this is the same idea:
hopping OFDM, a separate control channel, adaptive modulation, retransmission of what got
lost. Built from boards anyone can buy, with the source of the apps and the protocol here to
read and build on.

This is not open source. The source code in this repository is under the
[NyxHop Source Licence](LICENSE): read it, build it, change it, build your own applications
on it and sell them, as long as they run over a NyxHop link. The board images in `deploy/` are
binaries under a separate [EULA](docs/EULA.md), and the boards need a licence file to run -
the [Licensing](#licensing) table at the bottom says exactly which is which.

## How it fits together

```mermaid
flowchart LR
    CAM["Camera"] --> TX["nyx-tx<br/>PC or small SBC"]
    FC["Flight controller"] <-.-> TX
    TX -- "Ethernet" --> BA["SDR board"]
    BA == "radio link" ==> BB["SDR board"]
    BB -- "Ethernet" --> RX["nyx-rx<br/>PC or Android"]
    RX <-.-> GCS["Ground control station"]
```

Two boards, one at each end, each on Ethernet with the computer that runs its app. Video and
data go one way over the radio link, control and telemetry the other. Both boards run the same
image; which end a board plays is a flag when you flash it. The computers never touch the
radio: they exchange bytes with a board over Ethernet, and everything about the air interface
lives on the board.

## Getting started

Four steps: an OS on each board, NyxHop and an address on each board, the apps, a licence.
Budget an hour the first time.

**You need**

* Two boards, ADRV9364-Z7020 or ANTSDR E200 in any mix, with antennas for the band you will use.
  On the bench, 1–3 m apart is fine.
* A PC on the same Ethernet as the boards, with Python 3 (`pip install paramiko`) for the
  flashing tool. A USB camera on the transmitting side.
* This repository: it carries the prebuilt board images in `deploy/`. The apps you build
  yourself, with Rust 1.85 or newer: `cargo build --release` at the repository root, once, for
  all of them.

### 1. An OS on the board

**ADRV9364-Z7020**: write ADI's stock *Kuiper Linux 2023_r2* image to a microSD card (balenaEtcher,
or ADI's Kuiper Imager with the `adrv9364z7020_lvds` configuration), put it in the board, power up
with Ethernet connected. It takes an address by DHCP; user `root`, password `analog`. Find the
address on your router. Want a fixed one: give it a DHCP reservation on your router, or set a
static address on the board as on any Linux box.

**ANTSDR E200**: nothing to install. The stock firmware in its flash stays as it is; NyxHop boots
from a microSD card and the board is stock again the moment you take the card out. Put an empty
FAT32 card in and power up: a stock board comes up at `192.168.1.10`. Another address is one
command on the board, once: `ssh root@192.168.1.10`, then `fw_setenv ipaddr_eth 192.168.0.12`
and reboot.

### 2. NyxHop on the board

One command per board, from the repository root. `--host` is the board's address, `--role` is
which end it plays.

```bash
set BOARD_PW=analog        # the board's ssh password (PowerShell: $env:BOARD_PW = "analog")

# ADRV9364 as the transmitting end (camera side)
python link/scripts/nyx_flash.py --host 192.168.0.10 --role tx

# E200 as the receiving end (ground side)
python link/scripts/nyx_flash.py --host 192.168.0.12 --role rx --board e200
```

Any board can take either role. The tool copies the FPGA design, the radio daemon and its
configuration to the board (on the ADRV9364 over the Kuiper boot files, keeping the originals as
`*.nyx-orig`; on the E200 onto the card), reboots the board and checks that it came back. From
then on the board starts NyxHop by itself on every power-up. The addresses above are the ones
used throughout this page; yours are whatever your boards have.

Run the same command again to update a board; tables, pairing and licence are kept.
`--verify-only` only checks a running board. `--scan` lists every board it finds on the network,
with role and address, no password needed.

### 3. The apps

`cargo build --release` at the repository root puts them in `target/release/`. One app serves
either end:

```bash
nyxhop
```

It asks which end this computer is (**Ground station**: this computer shows the video;
**Aircraft**: this computer has the camera) and which board it is plugged into, puts the board
into the matching role (the board restarts, about ten seconds), and opens the ground or the
aircraft screen. It remembers the choice; `nyxhop --mode rx --board 192.168.0.12` or
`--mode tx --board 192.168.0.10` skips the question. So a pair of boards can change ends from
the apps alone: choose the other end on each computer, and both boards follow.

The settings drawer of every screen (the gear, or `H`) has a **Board role** switch too:
**Aircraft (tx)** / **Ground (rx)**. It sends the same command; the board restarts in its new
role, and a `nyxhop` window follows by opening the other screen on its own. Change both boards
and the link comes back the other way round, tables, pairing and licence untouched.

The two screens are also programs of their own, for scripts and headless boxes:

```bash
nyx-rx --channel 192.168.0.12:7011                # ground: the receiving board's address
nyx-tx --channel 192.168.0.10:7010                # aircraft: the transmitting board's address
nyx-tx --channel 192.168.0.10:7010 --headless     # no window, e.g. on an SBC next to the camera
```

The aircraft screen starts on the USB camera; with none plugged in it sends a test pattern.
*Source* in its settings also takes an **IP camera**: choose *IP camera (RTSP)* and type the
camera's URL, `rtsp://user:password@192.168.1.64:554/stream1` for instance (the address and
path are in the camera's manual; a 640x480 sub-stream is the right size). The camera's picture
is decoded and sent on like the webcam's, so the link's rate control applies to it.

**The Android app** is the same thing on a phone: it starts on the same choice, Ground station
or Aircraft, with the board's address, puts the board into the role and shows that end's
screen. As the ground end it shows the video; as the aircraft end it sends the phone's own
camera (the app asks for the camera permission once) or an IP camera (*Source* in the
settings, then the URL). The phone talks to the board over USB-C Ethernet or Wi-Fi. It
remembers the choice in `Android/data/com.nyxhop.mobile/files/nyxhop.cfg`, a plain text file
`adb` can edit (`board 192.168.0.12`, `mode tx`, `source phone|rtsp`, `rtsp <url>`).

The Android app is its own build: `python apps/video/build_apk.py` writes a signed APK to
`apps/video/android/out/`, and needs the Android SDK and NDK, `cargo-ndk` and the
`aarch64-linux-android` Rust target. You do not need it: the apps on a PC do the same job.

Then, once in the life of a pair of boards, **pair them**: press **Link aircraft** in the ground
app. A board that has never been paired accepts on its own, both pills turn *linked*, and the
video is on the ground screen. The boards remember the pairing across power cycles.

**Accept link** in the transmitting app is for the other case: an aircraft that is already
paired ignores a new key until someone at the aircraft presses it, which opens a 60-second
window. That is what stops a stranger from taking over your aircraft. **Unlink** forgets the
key on either end.

Everything else is in the apps' settings drawer (tap the video or press `H`):

* **Video** (aircraft): USB camera, IP camera (RTSP, a URL) or test pattern, resolution, fps,
  quality, codec, the simulcast base layer for reach through fades.
* **Channel**: your tables. *Video MHz* and *Control MHz* take any channels from 70 to 6000 MHz,
  comma separated; **Apply** sends them to the board, which passes them to the other end over
  the air. *Auto* holds the best channel and re-scans when it degrades; *Off* pins one channel.
  The tables are yours: check your local rules, fit antennas for the band.
* **Radio**: gains, antenna ports (the E200 has two per direction), AGC. Leave them alone unless
  you know why.
* **Messages**: short text between the ends. **Connection**: the board address.
* **Readouts** in the drawer header (or the `O` key) turns the figures over the video off,
  leaving the picture and the status pills. `H` opens and closes the drawer.

The headless transmitting app takes the same settings on its console (TCP 7201 by default):
`set source webcam`, `set source rtsp` with `set rtsp rtsp://...`, `set fps 20`,
`set quality 60`, `stats`; a `nyx-tx.cfg` next to the program holds them across starts.

**Telemetry and data**: the link also carries a two-way UDP pipe. Send datagrams into `nyx-rx`
UDP 14555 and they come out of `nyx-tx` UDP 14556 (commands, RC); send into `nyx-tx` UDP 14557
and they come out of `nyx-rx` UDP 14558 (telemetry). MAVLink is the usual payload: point your
ground control station at 14558 and your flight controller's MAVLink router at 14557. Details
and the SDK crates in [docs/telemetry-sdk.md](docs/telemetry-sdk.md).

### 4. The licence

Every board starts with a **20-hour grace period**, counted by the board itself, so you can do
all of the above first. Each board needs its own licence, and the simplest way is to do each
one in the app that is connected to it:

1. Open **Licence** in the drawer and press **Copy** next to the board's DNA.
2. Ask for a licence with it at **[nyxhop.com/licence.html](https://nyxhop.com/licence.html)**:
   free for the first ten boards per email address, commercial use included (see
   [docs/license.md](docs/license.md)). The file comes back on the page at once.
3. Paste it into the box under the DNA and press **Apply here**. The pill turns *licensed*.

Do that in `nyx-rx` for the ground board and in `nyx-tx` for the aircraft board. Once video is
flowing, the ground app also shows the aircraft's DNA and a **Send to aircraft** button, which
saves the walk to the aircraft next time; it needs the video link, so it is not there while the
aircraft is still locked. A licence is bound to its board, works offline and never expires.

### If something is off

* **The app cannot connect**: can you ping the board? Is the PC on the same network? Forgot
  the address: `python link/scripts/nyx_flash.py --scan`.
* **Connected but no video**: both pills must say *linked*; if not, press **Link aircraft** in
  the ground app again (and **Accept link** in the aircraft app if that board is already
  paired). A pill saying *locked* is a board past its grace period: licence it from the app
  connected to that board. Both ends must show the same tables: press **Apply** in Channel
  again. In the transmitting app, does the preview move? Try the test pattern to rule out the
  camera.
* **Video stutters**: SNR under 15 dB is antennas, distance or the wrong band; the pill shows
  *scan* while the receiver looks for a better channel.
* **Looking deeper**: the board console answers on TCP 7202 (`nc 192.168.0.12 7202`, then
  `hop status`, `license`, `stats`), the apps write logs next to the executable in `logs/`, and
  the daemon log is `/home/root/radio.out` (ADRV9364) or `/tmp/nyxhop/radio.out` (E200).

## Hardware

| board | transceiver | status |
|---|---|---|
| ADRV9364-Z7020 | AD9364 | ready |
| ANTSDR E200 | AD9361 | ready |
| PlutoSDR | AD9363 | in progress |

Either board can be either end, and two of the same kind work. Ethernet between each board
and its computer, 12 V supplies, antennas of your choice.

<img src="docs/img/bench.jpg" width="420" alt="An ADRV9364-Z7020 and an ANTSDR E200 on the bench">

*A pair on the bench: ADRV9364-Z7020 on the left, ANTSDR E200 on the right.*

The boards transmit at the power their own front end gives, which is enough for a bench and not
much more. For distance, add the RF yourself: an antenna with gain at both ends, a power
amplifier on the transmitting side, a low-noise amplifier on the receiving side. That part is
outside NyxHop, and the power you end up radiating is yours to keep within your local rules.

Another board, or one you would like to see on this list? Write to **tacitechvn@gmail.com**.

## General purpose, not a video product

The link itself knows nothing about video. It carries blocks and tells you which ones arrived.
Applications sit on top of it and each lives in its own folder:

| folder | what it does |
|---|---|
| `apps/video/` | the apps: H.264 from a USB, IP or phone camera one way, a ground app, a transmitting app and an Android app for either end. They also carry the two-way UDP pipe (MAVLink or any datagrams) |
| `apps/nyxhop/` | the one app for either end: asks Ground or Aircraft, sets the board's role, then hosts the screen |
| `apps/data/` | `mav_sim.py`, which measures that pipe |

Write your own the same way: take `nyx-proto` and `nyx-link` from `link/`, hand the link your
bytes, read them out at the far end. It is Rust with nothing platform-specific in the core:
Windows and Linux on x86-64, Linux on ARM boards, Android 12 or later. Nothing on the PC or the
phone needs a radio driver: the modem runs on the board.

## Pricing

The first ten boards per email address are free, commercial use included, and cover the current
feature generation with all its bug fixes, for ever. Licences are per board, not per pair, and
a board runs 20 hours before it needs one at all. Past ten boards, or for anything else -
another board, a feature you need, NyxHop inside something you sell - write to
**tacitechvn@gmail.com**. Details in [docs/license.md](docs/license.md).

## Custom work and support

Write to **tacitechvn@gmail.com** if you have a board we do not support yet, if you need a
feature or an application the link does not have, if you are putting NyxHop inside something
you sell, or if you want someone to tune it for your band, your range and your airframe.
Questions about using it as it is belong in Issues and Discussions, where everyone can read
the answer.

## Repository layout

```
link/         the transmission system: protocol, block framing, shared app support,
              board console, the flashing tool
apps/video/   the apps (ground, transmitting, Android) - video and the UDP data pipe
apps/data/    the tool that measures the pipe
deploy/       prebuilt board images you flash (deploy/adrv9364, deploy/e200)
docs/         licence terms, SDK notes, legal
```

## Licensing

| what | licence |
|---|---|
| Source code in this repository: `link/`, `apps/`, `docs/` | [NyxHop Source Licence](LICENSE): source-available, not open source |
| Binaries in `deploy/`: FPGA design, radio daemon, board images | [EULA](docs/EULA.md) |
| Third-party components inside the board images (U-Boot, BusyBox and others) | their own licences, see [THIRD-PARTY.md](docs/THIRD-PARTY.md) |

Radio use is subject to the laws of your country: you are responsible for the frequencies and the
power you transmit.
