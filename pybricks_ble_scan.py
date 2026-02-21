"""
pybricks_ble_scan.py

Raspberry Pi Pico 2W - MicroPython

This program listens for Bluetooth Low Energy (BLE) radio packets sent by
LEGO hubs running Pybricks firmware.
It prints each unique observed data to the terminal in a neat single-line format.

Each hub gets a tag: (A, B, C...) and a color so you can easily
follow which hub is which, even though hub addresses change randomly.
For hubs that started transmitting after THIS program has been started, also get the name
added to the data.

2026-02-21 BL Several updates to be able to serve more broadcasting hubs:
           The net effect:
           the queue drains 5× faster,
           removal is O(1) instead of O(n),
           address string formatting happens once per hub instead of once per packet,
           and scan response handling stays out of the hot path.
           This might handle 10 hubs at 250ms and likely at 100ms too.
"""
__version__ = "1.0.0"

# --- Performance notes ---
#
# Print throughput and Ctrl-C reliability:
#   MicroPython's print() is synchronous — it blocks until the UART buffer drains.
#   Under heavy test loads (many hubs broadcasting counters at 100ms)
#   the terminal can receive 10+ lines per second, causing visible stuttering
#   and making Ctrl-C unreliable (KeyboardInterrupt can only act between
#   Python bytecodes, not mid-print).
#   In real-world use this is not an issue:
#   a hub typically broadcasts one value that changes infrequently, so the
#   actual print rate is low even with many hubs.
#
# Packet throughput:
#   Tested with 7 hubs broadcasting at 100-250ms intervals (~85 packets/sec
#   Pybricks traffic).
#   Zero queue drops observed.
#   The deque queue, O(1) popleft(), 20ms main loop, and 25% scan duty cycle together provide
#   comfortable headroom for a potential 10-hub use case.
#
# PREVENTIVE_RESTART_EVENTS is an IRQ event count, not a time in seconds.
#   At ~120 ambient IRQ events/sec (phones, laptops, etc. nearby), a value
#   of 6000 gives roughly 50 seconds between preventive scan restarts.
#   Too low a value causes frequent restarts that can coincide with a hub's
#   boot-time name broadcast window, preventing name capture.
#
# Hub name capture:
#   City and Technic hubs include their name in regular advertisements —
#   captured immediately on first packet.
#   Move Hub sends its name only in scan responses shortly after boot —
#   captured if the observer is running when the hub boots, or on a
#   subsequent boot while the observer is running.
#   Prime Hub behaviour depends on firmware version.
#   All hubs: name fills in automatically mid-session when first received;
#   a notice line is printed at that moment so it is visible in the scroll.


import struct
import time
import bluetooth
from collections import deque

# --- Configuration - these to tune the program ---

# Set to True to only print a line when the value actually changes.
# Set to False to print every single packet received (can be a lot!).
SUPPRESS_DUPLICATES = True

# Color theme for the hub address in the terminal output.
# Use "dark"  if your terminal has a dark/black background.
# Use "light" if your terminal has a white/light background.
COLOR_THEME = "light"

# BLE scan parameters — these control how the radio scans for packets.
# duration_ms = 0 means scan forever (until we press Ctrl-C).
SCAN_DURATION_MS = 0

# interval_us is how often a scan window starts (in microseconds).
# window_us is how long each scan window lasts.
# 25% duty cycle (window = quarter of interval) keeps the BLE chip's
# internal event buffer from filling up under heavy traffic.
# Hubs broadcast repeatedly so we won't miss data at this duty cycle.
SCAN_INTERVAL_US = 100000   # 100 ms
SCAN_WINDOW_US   = 25000    # 25 ms  (25% duty cycle)

# active=False means we just listen — we never send scan requests back.
# This is called passive scanning. We don't want to talk, just observe.
# SCAN_ACTIVE      = False
SCAN_ACTIVE = True   # needed for hub name via scan response

# Watchdog: if no IRQ events arrive for this many seconds, restart the scan.
# The BLE chip can silently stop delivering events — this kicks it back.
WATCHDOG_SEC = 10

# Preventive restart: proactively restart the scan every N seconds to
# flush the BLE chip's internal event buffer before it fills up and stalls.
# Set to 0 to disable.

PREVENTIVE_RESTART_EVENTS = 6000    # was 120 — at ~120 IRQ events/sec ambient noise
                                 # 120 caused restarts every ~1s, colliding with
                                 # hub boot-time name packets. 6000 ≈ 50s between
                                 # restarts, giving hubs time to send their name.

# Debug mode: show heartbeat, watchdog, and restart messages.
# Set to False for clean output with data lines only.
DEBUG = True

DEBUG_RESTART = True  # Print message if scan has been restarted

# How often (in seconds) to print a heartbeat status line (when DEBUG==True).
HEARTBEAT_SEC = 30

"""
--- End configuration ---

How Pybricks encodes broadcast data in the BLE packet:
  After the manufacturer ID (0x0397) and channel number, each value
  is prefixed by a one-byte header: (type << 5) | length

  Type 0  SINGLE_OBJECT  header=0x00  (wraps a single value — recurse)
  Type 1  TRUE           header=0x20  no data bytes
  Type 2  FALSE          header=0x40  no data bytes
  Type 3  INT            header=0x61 (int8), 0x62 (int16), 0x64 (int32)
  Type 4  FLOAT          header=0x84  4 bytes IEEE 754 little-endian
  Type 5  STR            header=0xA0|len  UTF-8 text, len bytes
  Type 6  BYTES          header=0xC0|len  raw bytes, len bytes
"""

# The Pybricks company ID — every Pybricks BLE packet contains this.
# It is how we tell apart Pybricks packets from all other BLE devices.
PYBRICKS_COMPANY_ID = 0x0397

# AD type 0xFF means "manufacturer specific data" in the BLE standard.
AD_TYPE_MFR = 0xFF

# --- RSSI / signal strength ---
# Exponential moving average smoothing factor.
# 0.2 = 20% new reading, 80% history. Higher = more responsive, more jittery.
RSSI_EMA_ALPHA = 0.2

# RSSI thresholds and labels (dBm). Tuned for indoor room-scale BLE.
# Adjust RSSI_LEVELS if your environment differs.
RSSI_LEVELS = [
    (-55, "Very close"),   # same table
    (-70, "Nearby    "),   # same room
    (-80, "Far       "),   # across room
]
RSSI_WEAK   =  "Weak      "   # below all thresholds

# --- ANSI terminal color codes ---
# These are special escape sequences that tell the terminal to change
# the text color. \x1b[ starts the sequence, the number picks the color,
# and m ends it. \x1b[0m resets back to the default color.

# Colors that stand out on a dark/black terminal background
COLORS_DARK = [
    "\x1b[91m",   # bright red
    "\x1b[92m",   # bright green
    "\x1b[93m",   # bright yellow
    "\x1b[94m",   # bright blue
    "\x1b[95m",   # bright magenta
    "\x1b[96m",   # bright cyan
    "\x1b[31m",   # red
    "\x1b[32m",   # green
    "\x1b[33m",   # yellow
    "\x1b[35m",   # magenta
]

# Colors that stand out on a light/white terminal background
COLORS_LIGHT = [
    "\x1b[31m",   # red
    "\x1b[32m",   # green
    "\x1b[34m",   # blue
    "\x1b[35m",   # magenta
    "\x1b[36m",   # cyan
    "\x1b[33m",   # yellow/brown
    "\x1b[91m",   # bright red
    "\x1b[92m",   # bright green
    "\x1b[94m",   # bright blue
    "\x1b[95m",   # bright magenta
]

ANSI_RESET = "\x1b[0m"
COLORS     = COLORS_DARK if COLOR_THEME == "dark" else COLORS_LIGHT

# --- Hub registry ---
# We keep a dictionary that maps each hub's BLE address to its
# assigned letter tag and color. New hubs get the next available slot.
_hub_registry = {}   # address string -> (tag letter, ansi color string)
_name_cache = {}     # addr_bytes -> name, for scan responses arriving before
                     # the first manufacturer packet from that address.
                     # Entries are promoted to _hub_registry on first Pybricks packet.

_hub_counter  = [0]  # wrapped in a list so the IRQ function can modify it

# --- Deduplication state ---
# For each (address, channel) pair we remember the last value we printed.
# If the new value is the same, we skip printing it (when dedup is on).
_last_value = {}   # (address, channel) -> last decoded value string

# --- IRQ-safe packet queue ---
# The BLE interrupt handler (bt_irq) runs at a low level and must finish
# very quickly — it cannot do string formatting, printing, or dict lookups.
# So we just drop raw packet bytes into this list from the IRQ, and the
# main loop does all the heavy processing at its own pace.
#    Replace list queue with collections.deque — O(1) append and popleft,
#    and set a maxlen so overflow drops automatically without an if len() check in the IRQ:
# _queue = deque((), 60)   # maxlen=60, IRQ just appends — overflow drops oldest
_queue = deque((), 180)   # was  maxlen=60, IRQ just appends — overflow drops oldest

# --- Diagnostic counters ---
# Wrapped in single-element lists because Python closures can read outer
# variables but not reassign them — using a list lets us do _count[0] += 1
# from inside the IRQ function without a 'global' declaration.
_irq_count   = [0]   # how many BLE events the IRQ received in total
_queue_count = [0]   # how many of those were Pybricks packets we queued
_print_count = [0]   # how many lines we actually printed to the terminal
_t_start     = time.ticks_ms()   # program start time for elapsed display
_drop_count  = [0]   # packets lost to deque overflow (queue was full)
_processed_count = [0]
_intentional_restart = False


def rssi_label(dbm):
    """
    Convert an RSSI dBm value to a human-readable signal label.
    Thresholds are defined in RSSI_LEVELS; anything below the last
    threshold is returned as RSSI_WEAK.
    """
    for threshold, label in RSSI_LEVELS:
        if dbm >= threshold:
            return label
    return RSSI_WEAK


# In hub_tag_color — cache addr_string, add name slot
def hub_tag_color(addr):
    """
    Look up or assign a letter tag, color, and cached address string for a hub.
    The first hub we see gets tag 'A' and the first color, the second 'B', etc.

    addr_string is computed **once** here and reused every print
    — addr_str() is a string join and is not free to repeat 40 times per second.
    RSSI EMA and hub name are initialised to None on first sight.
    """
    if addr not in _hub_registry:
        idx        = _hub_counter[0] % len(COLORS)
        tag        = chr(ord('A') + (_hub_counter[0] % 26))
        addr_s     = addr_str(addr)
        #                      tag  color        ema   name  addr_string
        _hub_registry[addr] = (tag, COLORS[idx], None, None, addr_s)
        _hub_counter[0] += 1
    return _hub_registry[addr]


# update_rssi_ema — match new tuple shape
def update_rssi_ema(addr, rssi):
    """
    Update the exponential moving average RSSI for a hub.
    On first packet the EMA is seeded with the raw reading.
    Returns the updated EMA value.
    """
    tag, color, ema, name, addr_s = _hub_registry[addr]
    ema = rssi if ema is None else RSSI_EMA_ALPHA * rssi + (1 - RSSI_EMA_ALPHA) * ema
    _hub_registry[addr] = (tag, color, ema, name, addr_s)
    return ema


def update_hub_name(addr, name):
    """
    Store the hub's friendly name once we receive it in a scan response.
    Once set it is never overwritten — the name is stable for a hub's session.
    """
    tag, color, ema, existing, addr_s = _hub_registry[addr]
    if existing is None and name:
        _hub_registry[addr] = (tag, color, ema, name, addr_s)


# --- Pybricks value type encoding constants ---
# The top 3 bits of the header byte select the type.
# The bottom 5 bits carry the data length (for variable-length types).
_TYPE_MASK = 0b11100000
_LEN_MASK  = 0b00011111
_T_SINGLE  = 0 << 5   # 0x00
_T_TRUE    = 1 << 5   # 0x20
_T_FALSE   = 2 << 5   # 0x40
_T_INT     = 3 << 5   # 0x60
_T_FLOAT   = 4 << 5   # 0x80
_T_STR     = 5 << 5   # 0xA0
_T_BYTES   = 6 << 5   # 0xC0


def addr_str(addr):
    """Convert 6 raw address bytes into human-readable AA:BB:CC:DD:EE:FF format."""
    return ":".join(f"{b:02X}" for b in reversed(addr))


def find_local_name(payload):
    """
    Walk the BLE AD records looking for a Local Name record.
    AD type 0x08 = Shortened Local Name, 0x09 = Complete Local Name.
    Returns the name as a string, or None if not present in this payload.

    The name arrives in the scan response packet (adv_type 4), not the
    main advertisement, so it is only seen with SCAN_ACTIVE = True.
    The AD record structure is identical to find_mfr_offset:
      [length byte] [type byte] [data bytes...]
    where length includes the type byte but not the length byte itself.
    """
    i = 0
    while i < len(payload):
        length = payload[i]
        if length == 0:
            break                           # end of records
        i += 1
        if i + length > len(payload):
            break                           # malformed record, stop
        if payload[i] in (0x08, 0x09):      # shortened or complete local name
            try:
                return payload[i + 1:i + length].decode("utf-8")
            except Exception:
                return None                 # malformed UTF-8, skip
        i += length                         # skip to next record
    return None                             # no name record found


def find_mfr_offset(payload):
    """
    Walk through the BLE advertisement payload looking for the
    'manufacturer specific data' AD record (type 0xFF).
    Returns the byte offset of the first data byte after the type byte,
    or -1 if not found.

    BLE advertisements are made of AD records, each structured as:
      [length byte] [type byte] [data bytes...]
    where length includes the type byte but not the length byte itself.

    We verify the Pybricks company ID (0x97 0x03 little-endian)
    immediately after the 0xFF type byte before accepting the record.
    """
    i = 0
    while i < len(payload):
        length = payload[i]
        if length == 0:
            break                          # end of records
        i += 1
        if i + length > len(payload):
            break                          # malformed record, stop
        if payload[i] == AD_TYPE_MFR:
            # Verify Pybricks company ID (0x97 0x03 LE) before accepting.
            data_start = i + 1
            if (length >= 3 and
                    payload[data_start] == 0x97 and
                    payload[data_start + 1] == 0x03):
                return data_start          # confirmed Pybricks record
        i += length                        # skip to next record
    return -1                              # not found


def decode_value(data, pos):
    """
    Decode one Pybricks-encoded value from 'data' starting at byte 'pos'.
    Returns (decoded_python_value, number_of_data_bytes_consumed).
    Returns (None, 0) if decoding fails.
    """
    if pos >= len(data):
        return None, 0

    header = data[pos]
    typ    = header & _TYPE_MASK   # top 3 bits = type
    length = header & _LEN_MASK    # bottom 5 bits = data length
    pos   += 1                     # move past the header byte

    if typ == _T_SINGLE:
        # SINGLE_OBJECT is just a wrapper — the real value follows immediately.
        # Recurse to decode whatever comes next.
        return decode_value(data, pos)

    if typ == _T_TRUE:
        return True, 0

    if typ == _T_FALSE:
        return False, 0

    if typ == _T_INT:
        # Pick the right struct format based on the length field.
        # '<' means little-endian (least significant byte first).
        if length == 1:
            fmt, size = "<b", 1   # signed 8-bit integer
        elif length == 2:
            fmt, size = "<h", 2   # signed 16-bit integer
        elif length == 4:
            fmt, size = "<i", 4   # signed 32-bit integer
        else:
            return None, 0        # unexpected length
        if pos + size > len(data):
            return None, 0        # not enough bytes left
        value, = struct.unpack_from(fmt, data, pos)
        return value, size

    if typ == _T_FLOAT:
        # IEEE 754 single-precision float, always 4 bytes little-endian.
        if pos + 4 > len(data):
            return None, 0
        value, = struct.unpack_from("<f", data, pos)
        return value, 4

    if typ == _T_STR:
        # UTF-8 string — length bytes of text, no zero terminator.
        if pos + length > len(data):
            return None, 0
        return data[pos:pos + length].decode("utf-8"), length

    if typ == _T_BYTES:
        # Raw bytes — just return them as a bytes object.
        if pos + length > len(data):
            return None, 0
        return bytes(data[pos:pos + length]), length

    return None, 0   # unknown type


def print_detail_header():
    print(f"{'secs':>8}  {'Address':<17} [T] {'Hub name':<12} ch {'Signal':<18} {'Value'}")
    print("-" * 70)


def parse_pybricks(payload):
    """
    Try to parse a BLE advertisement payload as a Pybricks broadcast packet.
    Returns (channel, raw_data_bytes, decoded_string),
    or None if it is not a Pybricks packet.
    """
    # Find where the manufacturer data starts
    offset = find_mfr_offset(payload)
    if offset < 0:
        return None

    # We need at least 2 bytes company ID + 2 bytes channel + 1 header byte
    if offset + 5 > len(payload):
        return None

    # Read company ID (uint16 LE) and channel number (uint8).
    # Channel is a single byte (0-255). The original "<HH" read it as uint16,
    # which caused channel 255 (0xFF) to bleed into the next data byte,
    # producing garbage values like 42751 (0xA6FF) or 25087 (0x61FF).
    company_id, channel = struct.unpack_from("<HB", payload, offset)


    # Ignore packets from other manufacturers
    if company_id != PYBRICKS_COMPANY_ID:
        return None

    # data starts after company_id (2 bytes) + channel (1 byte)
    data_start = offset + 3
    data       = payload[data_start:]

    value, _ = decode_value(data, 0)

    # Format the decoded value as a string for display
    if value is None:
        decoded = "?"
    elif isinstance(value, bool):
        # Check bool before int — in Python, bool is a subclass of int!
        decoded = str(value)
    elif isinstance(value, float):
        decoded = f"{value:.6g}"
    elif isinstance(value, (bytes, bytearray)):
        decoded = value.hex()
    else:
        decoded = str(value)

    return channel, bytes(data), decoded


def contains_pybricks_id(mv):
    """
    Check if a memoryview contains the Pybricks company ID bytes 0x97 0x03.
    We cannot use 'in' for subsequence search on a memoryview in MicroPython,
    so we scan byte by byte. This is fast because BLE packets are at most 31 bytes.
    This pre-filter avoids copying and queuing packets from other devices.
    """
    for i in range(len(mv) - 1):
        if mv[i] == 0x97 and mv[i + 1] == 0x03:
            return True
    return False



def process_queue():
    """
    Process any packets waiting in the queue and print matching lines.
    This runs in the main loop where it is safe to do string operations,
    dictionary lookups, and printing — things too slow for the IRQ handler.
    Uses deque.popleft() for O(1) removal.
    """
    while _queue:
        _processed_count[0] += 1
        addr_bytes, payload, rssi, adv_type = _queue.popleft()
        
        # Try to extract a local name from any packet type —
        # City and Technic hubs include it in regular advertisements too.
        # This fills in the name on the first packet without waiting for
        # a scan response, and covers hubs that don't send scan responses.
        name = find_local_name(payload)
        if name and addr_bytes in _hub_registry:
            update_hub_name(addr_bytes, name)

# ===================================================
        if adv_type == 4:
            name = find_local_name(payload)
            if name:
                if addr_bytes in _hub_registry:
                    update_hub_name(addr_bytes, name)   # already known, store directly
                else:
                    _name_cache[addr_bytes] = name      # hold until first packet arrives
            continue
# ===================================================

#         # Scan response packets carry the hub name but no manufacturer data.
#         # Extract and store the name, then move on — nothing to print.
#         if adv_type == 4:
#             if addr_bytes in _hub_registry:
#                 name = find_local_name(payload)
#                 if name:
#                     update_hub_name(addr_bytes, name)
#             continue

        result = parse_pybricks(payload)
        if result is None:
            continue

        channel, _, decoded = result

        # Ensure registry entry exists before dedup/rssi lookup
        hub_tag_color(addr_bytes)

        # Promote cached name if one arrived in a scan response before
        # this first manufacturer packet.
        if addr_bytes in _name_cache:
            update_hub_name(addr_bytes, _name_cache.pop(addr_bytes))
            
        # Skip printing if value hasn't changed since last time (when dedup is on).
        # Key uses cached addr_string + channel to avoid tuple allocation per packet.
        tag, color, _, name, addr_s = _hub_registry[addr_bytes]
        if SUPPRESS_DUPLICATES:
            key = addr_s + str(channel)
            if _last_value.get(key) == decoded:
                continue
            _last_value[key] = decoded

        ema          = update_rssi_ema(addr_bytes, rssi)
        signal       = f"{rssi_label(ema)} {int(ema):4d}dBm"
        colored_addr = f"{color}{addr_s}{ANSI_RESET}"
        display_name = f" {name}" if name else ""
        _elaps       = time.ticks_diff(time.ticks_ms(), _t_start) // 1000
        _print_count[0] += 1
        print(f"{_elaps:8d}s {colored_addr} [{tag}]{display_name:<12} {channel:>3} {signal} {decoded}")


def bt_irq(event, data):
    """
    BLE interrupt handler — called by MicroPython whenever a BLE event occurs.
    This MUST be as fast as possible. No printing, no string formatting,
    no complex logic. Just check, copy the raw bytes, and get out.
    Scan responses (adv_type 4) are queued only if the hub is already known,
    to avoid filling the queue with name packets from unregistered devices.
    """
    if event == 5:   # _IRQ_SCAN_RESULT
        _, addr, adv_type, rssi, adv_data = data
        _irq_count[0] += 1

#         if adv_type == 4:   # scan response — queue for known hubs only
#             if bytes(addr) in _hub_registry:
#                 _queue_count[0] += 1
#                 _queue.append((bytes(addr), bytes(adv_data), int(rssi), int(adv_type)))
#             return

        if adv_type == 4:   # scan response — queue all, filter in process_queue
            _queue_count[0] += 1
            _queue.append((bytes(addr), bytes(adv_data), int(rssi), 4))
            return
        # Manufacturer advertisement — pre-filter for Pybricks ID
        if contains_pybricks_id(adv_data):
            _queue_count[0] += 1
            _queue.append((bytes(addr), bytes(adv_data), int(rssi), int(adv_type)))

    elif event == 6:   # _IRQ_SCAN_DONE
        if scan_is_running and not _intentional_restart:
            if DEBUG:
                print("  --- scan stopped unexpectedly, restarting ---")
            ble.gap_scan(SCAN_DURATION_MS, SCAN_INTERVAL_US, SCAN_WINDOW_US, SCAN_ACTIVE)


# --- Set up the BLE radio ---
ble = bluetooth.BLE()
ble.active(True)
ble.irq(bt_irq)   # register our interrupt handler

# --- State variables for the main loop ---
scan_is_running = True
_last_heartbeat = time.ticks_ms()
_last_irq_check = time.ticks_ms()
_last_irq_count = [0]   # snapshot of _irq_count at last watchdog/preventive check

# Start scanning
# gap_scan(duration_ms, interval_us, window_us, active)
ble.gap_scan(SCAN_DURATION_MS, SCAN_INTERVAL_US, SCAN_WINDOW_US, SCAN_ACTIVE)
print(f"pybricks-ble-observer v{__version__}  "
      f"Scanning for Pybricks BLE advertisements (Ctrl-C to stop)  "
      f"[dedup={'on' if SUPPRESS_DUPLICATES else 'off'}  "
      f"theme={COLOR_THEME}  debug={'on' if DEBUG else 'off'}]")

print_detail_header()

try:
    while scan_is_running:
        # Process any Pybricks packets that arrived since last loop
        process_queue()

        now = time.ticks_ms()

        # --- Preventive restart ---
        # Restart after PREVENTIVE_RESTART_EVENTS total IRQ events to flush the BLE
        # chip's internal buffer before it reaches the ~1100 event limit that stalls.
        # IRQ count is a reliable trigger because the stall is count-based, not time-based.
        if PREVENTIVE_RESTART_EVENTS > 0:
            if _irq_count[0] - _last_irq_count[0] >= PREVENTIVE_RESTART_EVENTS:
                elapsed = time.ticks_diff(now, _t_start) // 1000
                if DEBUG:
                    print(f"  --- preventive restart at {elapsed:6d}s  "
                          f"irq={_irq_count[0]}  queued={_queue_count[0]}  "
                          f"printed={_print_count[0]} ---")

                # global _intentional_restart
                _intentional_restart = True
                ble.gap_scan(None)
                ble.gap_scan(SCAN_DURATION_MS, SCAN_INTERVAL_US, SCAN_WINDOW_US, SCAN_ACTIVE)
                _intentional_restart = False
                _last_irq_count[0] = _irq_count[0]   # reset baseline
                _last_irq_check    = now             # reset watchdog too

        # --- Watchdog ---
        # Check every WATCHDOG_SEC seconds whether the IRQ is still firing.
        # If _irq_count hasn't moved, the BLE chip has gone silent unexpectedly —
        # restart to recover. Fallback for when preventive restart is off or
        # the stall happens before the preventive threshold.
        if time.ticks_diff(now, _last_irq_check) >= WATCHDOG_SEC * 1000:
            if _irq_count[0] == _last_irq_count[0]:
                elapsed = time.ticks_diff(now, _t_start) // 1000
                if DEBUG:
                    print(f"  --- watchdog {elapsed:6d}s: IRQ stalled, restarting ---")
                ble.gap_scan(None)
                ble.gap_scan(SCAN_DURATION_MS, SCAN_INTERVAL_US, SCAN_WINDOW_US, SCAN_ACTIVE)
            _last_irq_count[0] = _irq_count[0]
            _last_irq_check = now
            continue                              # skip watchdog check this iteration

        # --- Heartbeat ---
        # Periodically print a status line so we can see the program is alive.
        # Only shown when DEBUG = True.
        if time.ticks_diff(now, _last_heartbeat) >= HEARTBEAT_SEC * 1000:
            elapsed = time.ticks_diff(now, _t_start) // 1000
            drops = _queue_count[0] - _processed_count[0]
            if DEBUG:
                print(f"  --- heartbeat {elapsed:6d}s  "
                      f"irq={_irq_count[0]}  queued={_queue_count[0]}  "
                      f"processed={_processed_count[0]}  drops={drops}  "
                      f"printed={_print_count[0]}  qlen={len(_queue)} ---")

            _last_heartbeat = now
            print_detail_header()  # repeat the header for readability

        time.sleep_ms(20)   # was 100ms; tighter loop drains queue faster
                            # at 10 hubs × 250ms = 40 pkt/s, 20ms = ~1 pkt/iteration

except KeyboardInterrupt:
    scan_is_running = False
finally:
    # Clean up — stop the scan and turn off the BLE radio
    ble.gap_scan(None)
    ble.active(False)

    # --- Final statistics ---
    elapsed  = time.ticks_diff(time.ticks_ms(), _t_start) // 1000
    drops    = _queue_count[0] - _processed_count[0]
    hrs, rem = divmod(elapsed, 3600)
    mins, sec = divmod(rem, 60)
    print(f"\nScan stopped after {hrs:02d}:{mins:02d}:{sec:02d}")
    print(f"  BLE events received : {_irq_count[0]:>8d}")
    print(f"  Pybricks packets    : {_queue_count[0]:>8d}  ({100 * _queue_count[0] // max(_irq_count[0], 1)}% of events)")
    print(f"  Packets processed   : {_processed_count[0]:>8d}")
    print(f"  Deduped (suppressed): {_processed_count[0] - _print_count[0]:>8d}")
    print(f"  Lines printed       : {_print_count[0]:>8d}")
    print(f"  Queue drops         : {drops:>8d}{'  *** packets lost!' if drops > 0 else '  (none)'}")
    print(f"  Hubs seen           : {_hub_counter[0]:>8d}")
    for addr, entry in _hub_registry.items():
        tag, color, ema, name, addr_s = entry
        label = f" ({name})" if name else ""
        print(f"    [{tag}] {addr_s}{label}")


# Clean up — stop the scan and turn off the BLE radio
# ble.gap_scan(None)
# ble.active(False)
print("\nScan stopped.")

