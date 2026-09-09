# Telemetry, data pipe and SDK

Besides video, the link carries a two-way datagram pipe. Whatever you send into a UDP port on one
end comes out as the same datagram on the other end. MAVLink is the usual payload, but the pipe is
content-agnostic.

## The UDP pipe

| direction | into | out of | path over the air | use |
|---|---|---|---|---|
| ground → aircraft | `nyx-rx` UDP 14555 | `nyx-tx` UDP 14556 | control channel, small frames, repeated twice | RC, commands, parameters |
| aircraft → ground | `nyx-tx` UDP 14557 | `nyx-rx` UDP 14558 | video stream, retransmitted like video | telemetry, logs |

Datagrams up to 210 bytes each way. Change the ports with `--tlm-in` and `--tlm-out` on either app
(`--tlm-in 0` disables the input).

Measured on the bench: ground → aircraft 100 % of packets, aircraft → ground 98.6 % with 77 ms
median latency.

### Example: MAVLink to a ground-control station

Aircraft side, flight controller on a serial port → MAVLink router → UDP to `nyx-tx` port 14557.
Ground side, `nyx-rx` outputs on 14558 → point QGroundControl or Mission Planner at UDP 14558.
Commands go the other way through 14555 → 14556.

`mav_sim.py` in `apps/data/` generates real MAVLink heartbeats and RC overrides, sends them
through the pipe and reports loss and latency:

```bash
python mav_sim.py --dir tx2rx --rate 10 --duration 60
python mav_sim.py --dir rx2tx --rate 10 --duration 60
```

## Console API

Every component has a line-based TCP console: send a command, read lines until `ok` or `err …`.

| component | port | examples |
|---|---|---|
| board daemon | 7202 | `stats`, `hop status`, `hop chanq`, `hop chans …`, `hop auto rx 1000`, `hop manual 5815`, `license`, `link` |
| `nyx-rx` | 7203 | `stats`, `get` |
| `nyx-tx` | 7204 | `stats`, `set fps 20`, `set quality 60`, `set source webcam`, `set auto 1` |

`stats` prints `key=value` lines: frames, pictures decoded and failed, retransmissions, SNR, MCS,
bitrate, channel. Poll it from a script to log a flight.

## Crates for your own software

All Rust, MIT licensed, in `link/`:

* `nyx-proto` — the message protocol between the apps and the boards: video blocks, feedback,
  channel tables, licence messages, the UDP pipe. Enough to write your own ground station.
* `nyx-link` — segmentation of frames into payload blocks with CRC, and reassembly.
* `nyx-common` — video codec, sources and the shared touch UI.

The board daemon and the FPGA design, which do all the modem work, are binaries.

## Messages between the ends

The **Messages** section in the apps sends short text both ways over the same paths as the pipe.
