# Changelog

## 2.0.13

* NyxHop Pilot: flies with an RC transmitter plugged in by USB as a joystick (EdgeTX) or a
  gamepad. The motors start and stop with the sticks as on a camera drone, and the flight mode
  follows what the aircraft can hold: position, height only, or neither.
* NyxHop Pilot: Settings > Sensors: how the board sits in the frame, IMU, compass and level
  calibration, and live readings of a downward range finder and an optical flow sensor.
* NyxHop board (`fc/`): DShot300 by default, which BLHeli_S ESCs read reliably on the board, and
  TELEM1 at 115200 for an optical flow sensor speaking MAVLink. Run `fc/install.py install` again.
  The link and its boards are unchanged.

## 2.0.12

* ADRV9364: next to a busy WiFi channel the link no longer narrows to 2.5 MHz and stays there. It
  goes back to a wider channel as soon as that one does no worse, and it does not narrow for losses
  a narrower channel would not cure.
* ADRV9364: on some boards the video no longer drops for two seconds every half minute.
* ADRV9364: the link's distance reading is an extra now, off by default. Turn it on in the app
  (Range; in NyxHop Pilot: Distance without GPS) when you need it: the aircraft follows the ground
  station.
* ADRV9364: both ends start at full transmit power. With an external amplifier after the board,
  set its ceiling once: `tpc max -10`.
* NyxHop Pilot: a flight screen like a camera drone's app: ready to fly, or why not, in the
  flight controller's words; Cine, Normal and Sport; take off, land and return home held for a
  second; distance, height and speeds along the bottom; the aircraft's camera and transmit power
  in Settings, over the link.
* NyxHop board (`fc/`): DShot motors get the whole throttle range (they got half of it), and the
  ESC commands (spin direction, beeps) work from the console.
* Update both ADRV9364 boards and the camera program on the aircraft (`deploy/ipcam`). PlutoSDR
  is unchanged.

## 2.0.11

* ADRV9364 and PlutoSDR: telemetry and the IP network from the ground to the aircraft hold up
  when some of the control channels are busy (a cellular band nearby, for one): next to nothing is
  lost, large IP packets included, and a command to a flight controller is answered in about
  60 ms.
* Update both ADRV9364 boards, and flash the new `deploy/pluto/pluto.frm` on every Pluto.

## 2.0.10

* ADRV9364 and PlutoSDR: telemetry and the IP network from the ground to the aircraft work again.
  In 2.0.9 nothing sent from the ground reached the aircraft, so a flight controller behind the
  link took no commands.
* ADRV9364 and PlutoSDR: commands from the ground get through when some of the control channels
  are busy (a cellular band nearby, for one).
* ADRV9364 and PlutoSDR: after a restart the link goes back to its full width when some of its
  control channels are busy; it could stay at 2.5 MHz.
* Update both ADRV9364 boards, and flash the new `deploy/pluto/pluto.frm` on every Pluto.

## 2.0.9

* ADRV9364 and PlutoSDR: two NyxHop links side by side no longer mix. A ground could take the
  video of another pair's aircraft on its channel for its own, and showed it while its own
  aircraft was off. Update both ends of every pair; paired with an older release the link runs as
  before.
* ADRV9364 and PlutoSDR: after an outage the link widens back even when some of its control
  channels are busy (another link nearby, a cellular band).

## 2.0.8

* ADRV9364 and PlutoSDR: a control table set on the ground while the aircraft is off or out of
  range reaches the aircraft when it comes back, within about fifteen seconds of it starting up.
  The two used to stay apart until the aircraft was set by hand.
* ADRV9364 and PlutoSDR: after an outage the link goes back to its full width in one step when
  the signal allows it. An aircraft switched off and on a few times no longer keeps the link
  narrow for minutes, and an aircraft that restarts no longer finds the ground on a narrower
  width than the one it left.
* PlutoSDR: on a channel shared with Wi-Fi, the ground now sees that the aircraft is gone and
  narrows the link for it, as on a clear channel.
* Update both ADRV9364 boards, and flash the new `deploy/pluto/pluto.frm` on every Pluto.

## 2.0.7

* PlutoSDR: when either Pluto restarts or loses power for a moment, the link is back within about
  fifteen seconds of it starting up; it could take a minute, or not come back to the full width,
  before. After an outage the control link keeps its channels.
* PlutoSDR: the control link hops over up to 64 channels, up from 8, as on the ADRV9364 since
  2.0.4.
* Flash the new `deploy/pluto/pluto.frm` on every Pluto.

## 2.0.6

* ADRV9364: when either board restarts or loses power for a moment, the link is back within
  about ten seconds; it could take a minute or more before. Update both boards.
* ADRV9364: after an outage the control link keeps its channels; it used to drop most of them
  as jammed for a minute or two.

## 2.0.5

* ADRV9364: better resistance to interference. Bursts that hit the channel, Wi-Fi sharing it for
  one, now cost far fewer video frames. It needs 2.0.5 on both boards and turns itself on; paired
  with an older release the link runs as before.
* ADRV9364: a stronger signal at long range, while the fastest rates keep clear of overdriving the
  amplifier. A board with an external amplifier needs its transmit power capped for it
  (`tpc max -10` on that board's console, for example; the cap is remembered), or the extra power
  at long range pushes the amplifier into compression.
* ADRV9364: 20 running hours before a licence, down from 100. Boards that already have a licence
  are not affected.

## 2.0.4

* ADRV9364: the control link hops over up to 64 channels, up from 8. Set the table on the ground
  (`hop ctlchans`, or the app's channel panel); the aircraft gets it over the air. Channels that
  get jammed or busy are dropped on their own and tried again later.

## 2.0.3

* PlutoSDR: the link starts on Auto, channel and width, every time the Pluto is switched on, as
  the ADRV9364 since 2.0.2; a channel or a width picked by hand is no longer remembered. Flash the
  new `deploy/pluto/pluto.frm`.

## 2.0.2

* ADRV9364: the link starts on Auto, channel and width, every time the boards are switched on, and
  finds itself. A channel or a width picked by hand holds until the boards are switched off; it is
  no longer remembered.

## 2.0.1

* Lower video latency: on a clean channel a picture goes out in one go, about 30 ms from frame
  to screen at 720p. Its blocks are spread out only while the link sees interference.
* The camera app for the board (`deploy/ipcam/`, `link/scripts/nyx_ipcam_install.py`) runs on
  the 2.0 radio: an IP camera plugged into the aircraft board, no computer on board.

## 2.0

* A new radio on the ADRV9364-Z7020 and the PlutoSDR: up to 48 Mbit/s between two ADRV9364s in a
  20 MHz channel; channel widths of 20, 10, 5 and 2.5 MHz, and the link narrows by itself in a
  fade and widens again after.
* The link leaves a jammed channel by itself, and the ground screen says where it went
  ("CHANNEL a -> b MHz", "BANDWIDTH a -> b MHz").
* IP network over the link between two ADRV9364s: `link/scripts/nyx_ip.py on` joins a network on
  each side, next to the video.
* Video from a file: *Video file* in the aircraft app's sources (or drop a video on the window).
* `nyx-bitrate`, the link meter: the rate, the loss and the SNR as they come, and a sweep over
  every rate and width.
* One app, `nyxhop`, for either end.
* Licence: an ADRV9364 runs 100 hours before it needs a licence (counted only while it runs);
  the PlutoSDR stays free.
* The ADRV9364 and the PlutoSDR pair in any mix. The ANTSDR E200 stays on the previous radio:
  an E200 pairs with another E200.

## 1.0 (first public release, planned)

* Video link: H.264, adaptive bitrate and modulation, retransmission of lost frames, two-layer
  simulcast for the low end (off by default: it adds ~30 ms of latency; switch it on in the
  aircraft app for reach through fades). After a loss the aircraft repairs the picture with a
  small frame pointing at one the ground still holds, instead of a whole keyframe - measured on
  air, 544 bytes against 18553, so the picture comes back without the freeze a keyframe costs on
  the slow rungs ("Repair without keyframes" in the aircraft app).
* Faster back after a fade: the rate control no longer mistakes a deep fade for interference, reaches
  the slowest rate when it has to and climbs back in seconds (in a replayed fading channel: 13 % more
  pictures, a third of the lost ones). The ground's numbers keep moving while nothing arrives.
* More video on the same link: frames go out closer together on air (about 60 % more video at the
  same rate, no blocks dropped), the receiver copes better with weak signals (fewer lost frames at
  every level tried, about 1 dB at the middle rates), and at the slowest rate the link carries
  about five times the video it did, close to 30 fps, with the longest freeze halved.
* Back sooner after the signal drops out: full frame rate about 3 s after a 5 or 12 s outage (it
  took 9 to 14 s), and the link goes back to the channel it held before.
* Interference that hits only the fast rates: the link now measures the rate below and moves there
  instead of holding on, frames lost over 150 s down from 1536 to 340. Interference that hits every
  rate alike, like WiFi, is handled as before.
* Distance readout steady to about 0.3 m (it wandered by ±9 m) and no jump after a power cycle;
  about 5 ms less glass-to-glass delay.
* Channels: user-defined tables over 70 MHz–6 GHz, Auto policy (hold the best channel, re-scan
  when it degrades or the link drops), fixed-channel mode, tables sent to the aircraft over the air.
* Control link on its own channel pool, frequency hopping in 125 ms slots, paired links with a
  key per pair, clock fallback when the aircraft loses the receiver.
* Telemetry: two-way UDP pipe (MAVLink or any datagrams).
* Apps: one app for either end (`nyxhop`: choose Ground or Aircraft, the board takes the matching
  role), the ground and aircraft screens as programs of their own (Windows / Linux, headless),
  an Android app for either end (the phone's camera or an IP camera as the aircraft source).
  The aircraft end on a PC takes a USB camera or an IP camera (RTSP): the camera's own H.264 can
  go on air untouched, and the camera's bitrate can follow the link over ONVIF. A Board role
  switch in every settings drawer turns a board into the other end; the apps follow it.
* The camera straight into the board: `nyx-ipcam` runs on the board's own ARM and sends an IP
  camera with no computer on the aircraft (prebuilt in `deploy/ipcam/`, one-command installer,
  a PC window to set it up, a test camera). Either board, follows the board's role.
* Boards: ADRV9364-Z7020 on ADI Kuiper Linux (one-command installer), ANTSDR E200 (SD card),
  PlutoSDR (one file copied onto its USB drive, or one command for every Pluto plugged in:
  firmware, a network of its own, pairing; free, no licence). Any board takes either end, in any
  mix. A PlutoSDR tunes its frequency to the board at the other end by itself.
* Licence per board, checked offline; free tier.
* Licensing of the repository: the source code is public domain (Unlicense), no licence needed;
  the board images stay under the EULA.
