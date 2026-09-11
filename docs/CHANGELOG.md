# Changelog

## 1.0 (first public release, planned)

* Video link: H.264, adaptive bitrate and modulation, retransmission of lost frames, two-layer
  simulcast for the low end (off by default: it adds ~30 ms of latency; switch it on in the
  aircraft app for reach through fades).
* Channels: user-defined tables over 70 MHz–6 GHz, Auto policy (hold the best channel, re-scan
  when it degrades or the link drops), fixed-channel mode, tables sent to the aircraft over the air.
* Control link on its own channel pool, frequency hopping in 125 ms slots, paired links with a
  key per pair, clock fallback when the aircraft loses the receiver.
* Telemetry: two-way UDP pipe (MAVLink or any datagrams).
* Apps: one app for either end (`nyxhop`: choose Ground or Aircraft, the board takes the matching
  role), the ground and aircraft screens as programs of their own (Windows / Linux, headless),
  an Android ground app. The aircraft end takes a USB camera or an IP camera (RTSP). A Board
  role switch in every settings drawer turns a board into the other end; `nyxhop` follows it.
* Boards: ADRV9364-Z7020 on ADI Kuiper Linux (one-command installer), ANTSDR E200 (SD card).
* Licence per board, checked offline; free tier.
