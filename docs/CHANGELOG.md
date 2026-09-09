# Changelog

## 1.0 (first public release, planned)

* Video link: H.264, adaptive bitrate and modulation, retransmission of lost frames, two-layer
  simulcast for the low end.
* Channels: user-defined tables over 70 MHz–6 GHz, Auto policy (hold the best channel, re-scan
  when it degrades or the link drops), fixed-channel mode, tables sent to the aircraft over the air.
* Control link on its own channel pool, frequency hopping in 125 ms slots, paired links with a
  key per pair, clock fallback when the aircraft loses the receiver.
* Telemetry: two-way UDP pipe (MAVLink or any datagrams).
* Apps: ground app (Windows), Android app, aircraft app (Windows / Linux, headless).
* Boards: ADRV9364-Z7020 on ADI Kuiper Linux (one-command installer), ANTSDR E200 (SD card).
* Licence per board, checked offline; free tier.
