# JK BMS over RS485 (Modbus RTU)

CAN can read everything, but it can only switch CHG, DSG and the balancer.
RS485 can do more: shut the BMS down, and read or write every setting.

Tested on a **JK-B2A24S30P**, hardware `19H`, software `19.31`
(the "V19" generation), with a plain USB-RS485 dongle.

## Wiring

| JK BMS | RS485 dongle |
|---|---|
| connector P5, pin 3 / pin 4 | A / B (swap them if nothing answers) |

The same P5 connector also carries CAN on pins 1 (CAN_L) and 2 (CAN_H).
On this pack **CAN and RS485 worked at the same time**.

## Link settings

- **Modbus RTU, 115200 baud, 8N1, slave address 1.**
- Use function **0x10** (write multiple registers) for every write.
  Function 0x06 is **not supported**: the BMS answers with exception 01.
- The older JK "4E 57" protocol gets **no reply** on this board.

## Useful registers

| Address | What |
|---|---|
| `0x1400` | device info block: model text (`JK-B2A24S30P`), hardware, software version |
| `0x1420` | runtime in seconds (UINT32) |
| `0x1424` | power-on count |
| `0x1600` + 4 = `0x1604` | shutdown command (write 1) |

## Shut the BMS down

Send:

```
01 10 16 04 00 01 02 00 01 11 D5
```

That is function 0x10, register `0x1604`, one register, value 1, then the CRC.
The BMS echoes the command and goes silent within about 4 seconds.

To wake it up, press the button on P8 (K+ / K-) or connect a charger.

There is **no restart command** over Modbus. JK's own app restarts the BMS
over BLE.

## Read and write settings

Settings live in a block that starts at `0x1000`.

- A setting's address is the block base **plus its byte offset**.
  Example: offset `0x34` is address `0x1034`.
- Values are **UINT32, big-endian**.
- Read with function 0x03, write with function 0x10.

Exporting every setting to a file and importing it again works. Always save
an export before you change anything.

## What each link can do

| Link | Read | Switch CHG / DSG / BAL | Settings | Shutdown | Restart |
|---|---|---|---|---|---|
| CAN (BMS-CAN V2.1, 250 kbit/s) | yes | yes (`18F0F428`) | no | no | no |
| RS485 Modbus RTU | yes | yes | yes | yes | no |
| BLE (JK app) | yes | yes | yes | yes | yes |
