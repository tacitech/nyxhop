# NyxHop baseboard v0.2

The board the radio module plugs into: ADRV9364-Z7020, or ADRV9361-Z7035 for 2x2. 100 × 62 mm,
powered from a 2S–6S battery (7–26 V).

Schematic: [nyxhop-baseboard-v0.2-schematic.pdf](nyxhop-baseboard-v0.2-schematic.pdf)

## Ports

JST-GH with the Pixhawk DS-009 pin order unless noted; pin 1 is the red wire. Signals are 3.3 V and
5 V tolerant.

| Port | Connector | Pins |
|---|---|---|
| TELEM1, TELEM2 | GH 6P | 1 5V, 2 TX, 3 RX, 4 CTS, 5 RTS, 6 GND |
| GPS1 | GH 10P | 1 5V, 2 TX, 3 RX, 4 SCL, 5 SDA, 6 SAFETY_SW, 7 SAFETY_LED, 8 3V3 out, 9 BUZZER, 10 GND |
| GPS2 | GH 6P | 1 5V, 2 TX, 3 RX, 4 SCL, 5 SDA, 6 GND |
| I2C | GH 4P | 1 5V, 2 SCL, 3 SDA, 4 GND (same bus as GPS2) |
| SPI | GH 7P | 1 5V, 2 SCK, 3 MISO, 4 MOSI, 5 CS1, 6 CS2, 7 GND |
| RC IN | GH 5P | 1 5V, 2 RC_IN (CRSF, SBUS or PPM), 3 RSSI (analog), 4 RC_TX (CRSF telemetry), 5 GND |
| CAN1, CAN2 | GH 4P | 1 5V, 2 CAN_H, 3 CAN_L, 4 GND |
| PWM OUT | GH 10P | 1 not connected, 2–9 CH1–CH8, 10 GND |
| ESC (4-in-1) | JST-SH 8P | 1 VBAT, 2 GND, 3 CURR, 4 TLM, 5–8 M1–M4 (same as CH1–CH4) |
| ETH | GH 4P | 1 TX+, 2 TX−, 3 RX+, 4 RX− (100 Mbit/s) |
| DEBUG | JST-SH 10P | 1 3V3, 2 CONSOLE_TX, 3 CONSOLE_RX, 4 TMS, 5 TCK, 6 TDO, 7 TDI, 8 NC, 9 nRST, 10 GND |
| RF-CTL | GH 8P | 1 5V, 2–5 GPO0–GPO3, 6 AUXDAC1, 7 AUXADC, 8 GND (for a PA/LNA module; not fitted by default) |
| 5V AUX | GH 2P | 1 5V (1.4 A), 2 GND (an IP camera or a companion computer) |
| Power | XT30 | battery, 2S–6S |

VBAT on the ESC port does not power the board. DEBUG pins 1, 2, 3 and 10 follow the Pixhawk Debug
Full layout, so a Pixhawk debug adapter or any 3.3 V USB-serial cable gives the console.

Also on the board: USB-C (OTG), microSD, an ICM-42688-P IMU with a heater, battery voltage and
current sensing, an RGB LED and a reset button.
