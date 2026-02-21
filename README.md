# pybricks-ble-observer

**BLE observer for Pybricks LEGO hub broadcasts — MicroPython on Pico 2W**

Listens for Bluetooth Low Energy (BLE) broadcast packets sent by LEGO hubs
running [Pybricks](https://pybricks.com) firmware, and prints each unique
value to the terminal in a single-line format.

```
    secs  Address           [T] Hub name     ch Signal             Value
  ----------------------------------------------------------------------
       0s 4D:FB:94:3F:B2:04 [A] city 4736     5 Far         -79dBm 42
       0s 2A:38:C0:12:D5:07 [B] hubA beta     5 Nearby      -66dBm hello
       1s 93:B2:F6:E1:6C:0D [C] hubc beta     5 Weak        -83dBm (1, 2, 3)
       1s ED:D9:BD:44:5B:F5 [D]               5 Far         -78dBm True
```

Each hub gets a letter tag and a color so you can follow multiple hubs at a
glance, even though BLE addresses change on every power cycle. 
Hub names are captured automatically from the BLE advertisement where available.

---

## Hardware requirements

- **Raspberry Pi Pico 2W** (the wireless variant with CYW43439 BLE chip)
- USB connection to a computer running a terminal (Thonny, PuTTY, etc.)
- One or more LEGO hubs running Pybricks firmware and using `hub.ble.broadcast()`

The program runs entirely on the Pico 2W — no host-side software beyond a
serial terminal needed.

---

## Software requirements

- **MicroPython** for Pico 2W 
    — tested with MicroPython v1.26.0-preview.265.ge57aa7e70 on 2025-06-23; Raspberry Pi Pico W with RP2040

- No additional libraries required — uses only `bluetooth`, `struct`, `time`,
  and `collections.deque` from the MicroPython standard library

---

## Installation

1. Flash MicroPython onto your Pico 2W if you haven't already —
   see [micropython.org/download/RPI_PICO2_W](https://micropython.org/download/RPI_PICO2_W/)
2. Copy `pybricks_ble_scan.py` to the Pico 2W using Thonny
3. Open a serial terminal to the Pico 2W (like e.g. Thonny) and run the file

---

## Usage

Run `pybricks_ble_scan.py` on the Pico 2W. The program scans indefinitely
until you press **Ctrl-C**.

```
Scanning for Pybricks BLE advertisements (Ctrl-C to stop)  [dedup=on  theme=light  debug=on]
    secs  Address           [T] Hub name     ch Signal             Value
----------------------------------------------------------------------
       2s 4D:FB:94:3F:B2:04 [A] city 4736     5 Far         -79dBm 1024
       3s 2A:38:C0:12:D5:07 [B] hubA beta     5 Nearby      -66dBm 2048
```

On exit, a statistics summary is printed:

```
Scan stopped after 00:03:06
  BLE events received :     6982
  Pybricks packets    :     5867  (84% of events)
  Packets processed   :     5867
  Deduped (suppressed):     3481
  Lines printed       :     2386
  Queue drops         :        0  (none)
  Hubs seen           :        8
    [A] 7F:3E:7F:CE:80:01 (hubD 4577)
    [B] 93:B2:F6:E1:6C:0D (hubc beta)
    ...
```

---

## Configuration

All tunable settings are at the top of the file with inline comments.
The most useful ones:

| Setting | Default | Description |
|---|---|---|
| `SUPPRESS_DUPLICATES` | `True` | Only print when value changes |
| `COLOR_THEME` | `"light"` | `"light"` or `"dark"` terminal background |
| `SCAN_ACTIVE` | `True` | Active scanning to capture hub names |
| `SCAN_INTERVAL_US` | `100000` | Scan interval (100 ms) |
| `SCAN_WINDOW_US` | `25000` | Scan window (25 ms = 25% duty cycle) |
| `WATCHDOG_SEC` | `10` | Restart scan if IRQ goes silent |
| `PREVENTIVE_RESTART_EVENTS` | `6000` | Restart scan every N IRQ events* |
| `DEBUG` | `True` | Show heartbeat and restart messages |
| `HEARTBEAT_SEC` | `30` | Heartbeat interval in seconds |
| `RSSI_EMA_ALPHA` | `0.2` | RSSI smoothing (0=none, 1=raw) |
| `RSSI_LEVELS` | see file | dBm thresholds for signal labels |

\* `PREVENTIVE_RESTART_EVENT` is an **IRQ event count**, not a time in seconds.
At ~120 ambient IRQ events/sec, a value of 6000 gives roughly 50 seconds
between restarts. The name is kept for historical reasons.

---

## Output columns

| Column | Description |
|---|---|
| `secs` | Elapsed seconds since program start |
| `Address` | BLE MAC address of the hub |
| `[T]` | Letter tag assigned to this hub (A, B, C…) |
| `Hub name` | Friendly name from BLE advertisement (if available) |
| `ch` | Pybricks broadcast channel number (0–255) |
| `Signal` | RSSI label + smoothed dBm value |
| `Value` | Decoded broadcast value |

---

## Supported value types

Pybricks can broadcast any of these Python types and this observer decodes
all of them:

| Type | Example output |
|---|---|
| `int` | `42` |
| `float` | `3.14159` |
| `bool` | `True` |
| `str` | `hello` |
| `bytes` | `deadbeef` (hex) |
| `list` | `[1, 3, 5, 7]` |
| `tuple` | `(tuple1, tuple2)` |

---

## Hub name support

Hub names are captured automatically from BLE advertisements 
and scan responses — no configuration needed.

| Hub type | Name source | Reliability |
|---|---|---|
| City Hub | Regular advertisement | Immediate, every session |
| Technic Hub | Regular advertisement | Immediate, every session |
| Move Hub | Scan response at boot | Captured if observer running at hub boot |
| Prime Hub | Scan response | Firmware-dependent |

Hubs with random BLE addresses (all current Pybricks hubs) get a new address
on every power cycle, so names cannot be pre-configured by address.

---

## Performance

Tested with 7 hubs broadcasting simultaneously at 100–250 ms intervals
(~85 Pybricks packets/second):

- Zero queue drops observed over 3+ minute sessions
- Comfortable headroom for the 10-hub use case
- Print throughput is the practical bottleneck at very high broadcast rates as tested.
  But in real live usage a hub will broadcast a value that changes infrequently.

The BLE chip is scanned at 25% duty cycle (25 ms window / 100 ms interval)
to prevent the internal event buffer from stalling. 
A preventive scan restart and a watchdog provide additional robustness
for long-running sessions.

---

## Broadcaster side (minimal example)

Any Pybricks hub can broadcast with:

```python
from pybricks.hubs import ThisHub
from pybricks.tools import wait

hub = ThisHub()

counter = 0
while True:
    hub.ble.broadcast(counter)
    counter += 1
    wait(250)
```

---

## License

MIT — see [LICENSE](LICENSE)

## Acknowledgements

Built for the [Pybricks](https://pybricks.com) community.

Pybricks BLE broadcast protocol documentation:
[github.com/pybricks/technical-info](https://github.com/pybricks/technical-info)
