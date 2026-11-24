# jk-bms-can-esp32

Reads a JK BMS over CAN and can switch its charge MOSFET, discharge MOSFET
and balancer. ESP32-S3, ESP-IDF 6.0. Web page, OTA, mDNS.

The protocol is JK **BMS-CAN V2.1**. JK's own protocol documents are not copied here.
Get them from JK: <https://github.com/NEEY-electronic/JK> (branch `JK-BMS`).

What CAN cannot do (restart, settings) works over RS485 Modbus instead:
see [docs/RS485_MODBUS.md](docs/RS485_MODBUS.md).


## Wiring

| ESP32-S3 | CAN transceiver |
|---|---|
| GPIO17 | TXD (sometimes marked CTX) |
| GPIO18 | RXD (sometimes marked CRX) |
| 5V or 3V3 | VCC — see below |
| GND | GND |

| Transceiver | JK BMS, connector P5 |
|---|---|
| CAN_H | pin 2 (CAN_H) |
| CAN_L | pin 1 (CAN_L) |

P5 pin 1 is **CAN_L** and pin 2 is **CAN_H**. Many people expect the other
order.

**Three things that stop the CAN link from working:**

1. **The transceiver needs power.** With no VCC, RX reads LOW all the time.
   The web page then shows `RX pin low 1000/1000`.
2. **5 V on RX damages the ESP32-S3.** Its pins are not 5 V tolerant.
   - TJA1050 or MCP2551 (5 V chips): put a divider in the RX line, 1k from
     RXD and 2k to GND. TX from the ESP (3.3 V) is fine as it is.
   - SN65HVD230 (3.3 V chip): power it from **3V3**, no divider.
3. **Termination and ground.** Put 120 Ω between CAN_H and CAN_L at **both**
   ends of the bus. With power off, you should measure about **60 Ω**
   across H–L. Also join the ESP's GND to the pack's **B−**. Without
   these, the bus is full of `form` errors and most frames are lost.


## First flash (USB, once)

```
cp sdkconfig.defaults.local.example sdkconfig.defaults.local   # then fill it in
idf.py set-target esp32s3
idf.py build flash monitor
```

The partition table and the rollback setting can only change over USB.
After this, use OTA.

`sdkconfig.defaults.local` holds the Wi-Fi password, the access point
password and the OTA token. It is not in git. If you change it after the
first build, delete `sdkconfig` so the new values are read.


## Open the page

- On the house network: `http://jkbms.local/` or the IP in the log.
- Anywhere else: join the Wi-Fi **JKBMS-CAN** (password in
  `sdkconfig.defaults.local`), then open `http://192.168.4.1/`.

The access point is off if `CONFIG_JKB_AP_PASS` is empty. It is never open
without a password, because the page can turn the bike off.


## Firmware update over Wi-Fi

```
idf.py build
curl --data-binary @build/jk_bms_can.bin http://jkbms.local/api/ota
```

Or pick `build/jk_bms_can.bin` in the page's **Firmware update** box.

The board checks the file first (ESP image, ESP32-S3, project `jk_bms_can`),
so a wrong .bin is refused before anything is erased. The new firmware
boots **on trial**. If it resets in the first 30 s, the old firmware comes
back by itself.


## API

| Method | Path | What |
|---|---|---|
| GET | `/api/state` | JSON: firmware, Wi-Fi, CAN diagnostics, all BMS values |
| GET | `/api/can` | text: id table and the newest 40 frames |
| POST | `/api/ctrl?chg=1` | charge MOSFET on (`0` = off) |
| POST | `/api/ctrl?dsg=0&sure=1` | discharge MOSFET off. **Bike turns off.** Needs `sure=1` |
| POST | `/api/ctrl?bal=1` | balancer on |
| POST | `/api/send?id=18F0F428&data=0400000100000000` | any frame. Ids longer than 3 digits are extended |
| POST | `/api/rate?kbps=250` | 125, 250, 500 or 1000. Saved |
| POST | `/api/clear` | empty the id table |
| POST | `/api/ota` | body = firmware .bin |

Example: `curl -X POST "http://jkbms.local/api/ctrl?chg=0"`


## The protocol (V2.1, 250 kbit/s)

BMS address `0xF4` (+ the device address from the JK app, `CONFIG_JKB_BMS_ADDR`).
Little-endian, except `1806E5F4`.

| ID | Type | Every | What |
|---|---|---|---|
| `2F4` | std | 20 ms | pack V (0.1 V), current (0.1 A, −400 A offset, **+ = charging**), SOC % |
| `4F4` | std | 100 ms | max cell mV + cell no, min cell mV + cell no |
| `5F4` | std | 500 ms | max / min / average temperature (°C + 50), sensor numbers |
| `7F4` | std | 100 ms | alarms, 2 bits each. **Only sent while an alarm is on** |
| `18F128F4` | ext | 100 ms | remaining Ah, full Ah, cycled Ah (all 0.1 Ah), cycle count |
| `18F228F4` | ext | 500 ms | sensor mask + up to 5 temperatures (°C + 50, `FF` = none). T3 = MOSFET |
| `18F328F4` | ext | 100 ms | 18 fault bits |
| `18F428F4` | ext | 500 ms | BMS runtime (s), heater current (mA), SOH % |
| `18F528F4` | ext | 500 ms | bit0 CHG, bit1 DSG, bit2 balancing now, bit3 heater, bit4 charger in, bit5 ACC |
| `18E0..18E628F4` | ext | 1000 ms | cell voltages, 4 per frame, mV |
| `1806E5F4` | ext | 500 ms | charge request, **big-endian**: V, A (0.1), charger off, heat mode. Only with a charger |

**The one frame we send — Ctrl_INFO `18F0F428`:**

| Byte | Meaning |
|---|---|
| 0 | mask: bit0 = apply CHG, bit1 = apply DSG, bit2 = apply BAL |
| 1 | CHG: 0 off, 1 on |
| 2 | DSG: 0 off, 1 on |
| 3 | BAL: 0 off, 1 on |

Only the switches in the mask change. The firmware sets the mask for you.
There is **no restart and no settings write** over CAN. For those, use
RS485 Modbus ([docs/RS485_MODBUS.md](docs/RS485_MODBUS.md)) or the JK app over BLE.

After a CHG or DSG write, the firmware watches `18F528F4`. Within 3 s the
page says **CONFIRMED** or **NOT confirmed**. A balancer write cannot be
confirmed, because bit 2 means "balancing now", not "balancer enabled".


## Reading the CAN diagnostics

The log prints this every 10 s, and the page shows the same numbers:

```
can: 250 kbit/s ok  26/s  rx 1234  tx 0/0 fail  tec 0 rec 0  rxpin low 12/1000  err bit 0 ack 0 stuff 0 form 0
```

| You see | It means |
|---|---|
| `rxpin low 1000/1000`, no errors | RX held LOW: transceiver has no power, or the bus is held dominant |
| `rxpin low ~0`, `form`/`stuff` climbing | edges but no frames: noise (no termination) or wrong bit rate |
| `rxpin low` 30–80 %, huge `form` | bus mostly dominant: missing termination or GND |
| `ack` climbing when you send | nobody ACKed: nothing else on the bus at this bit rate |
| `26/s`, errors not climbing | healthy. JK sends about 25–30 frames a second |

Every 10 s the log also prints one BMS line, or `BMS offline` if nothing
came from address 0xF4. It never goes silent.


## Seen on the bench, 2026-09-29

From a JK-B2A24S30P pack (hardware 19H, software 19.31): 20S LFP, 68.1 V, 59 %, 17.6 / 30.0 Ah,
DSG on, CHG off. `01F21400` also appeared, from source address `00`. It is
not in the JK document. It may be damaged frames from the bad bus; check
again on a clean bus.


## License

MIT, see [LICENSE](LICENSE).
