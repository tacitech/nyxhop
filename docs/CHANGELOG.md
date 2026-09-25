# Changelog

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
* Back sooner after the signal drops out: on the bench, full frame rate about 3 s after a 5 or 12 s
  outage (it took 9 to 14 s), and the link goes back to the channel it held before.
* Interference that hits only the fast rates (a neighbouring LTE band on the bench): the link now
  measures the rate below and moves there instead of holding on, frames lost over 150 s down from
  1536 to 340. Interference that hits every rate alike, like WiFi, is handled as before.
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
  a PC window to set it up, a test camera for the bench). Either board, follows the board's role.
* Boards: ADRV9364-Z7020 on ADI Kuiper Linux (one-command installer), ANTSDR E200 (SD card).
* Licence per board, checked offline; free tier.
* Licensing of the repository: the source code is public domain (Unlicense), no licence needed;
  the board images stay under the EULA.
