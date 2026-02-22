# Changelog

All notable changes to this project will be documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [1.0.1] - 2026-02-22

### Pylint updates

2026-02-22 BL Code review and refactor:
  Fix pylint errors and warnings

  test programs:
  - ble_broadcast_counter
      wait to 500ms to better observe ALL counters 
      channel to 3 to better see the differences

  process_queue:
  - Eliminated duplicate find_local_name() call — payload was scanned twice
    for scan response packets (adv_type == 4)
  - Extracted _format_line() as module-level helper to avoid reallocation
    on every packet in the hot path
  - Clarified name handling logic: scan response and regular advertisement
    paths are now clearly separated

  parse_pybricks:
  - Removed raw_data_bytes from return value — never used by any caller
  - Extracted _format_decoded() helper to separate value formatting
    from BLE packet parsing
  - Updated docstring to reflect new 2-tuple return

  decode_value:
  - Eliminated recursion for _T_SINGLE — replaced with while loop
    to avoid stack frame allocation on MicroPython
  - Extracted _decode_int(), _decode_float(), _decode_str(), _decode_bytes()
    as module-level helpers for readability
  - _decode_float: removed unused length parameter
  - Added pylint disable for too-many-return-statements (9/6)

  General:
  - Removed dead _TYPE_DISPATCH dict and lambdas
  - Removed dead _drop_count variable — drop count derived from
    _queue_count vs _processed_count which is already reported
  - Removed stale commented-out code (decode_value_OLD, old parse_pybricks block)
  - Removed stale # global _intentional_restart comment
  - Moved _INT_FMTS dict to module level — was being rebuilt on every
    _decode_int call
  - Normalised local variable naming in _format_line to trailing underscore
    convention throughout
  - Expanded SCAN_WINDOW_US documentation with duty cycle options and
    guidance for single-hub vs multi-hub use cases

  Testing:
  - Verified 100% duty cycle improves packet capture significantly
  - At 250ms broadcast interval, ~4-6 receptions per broadcast confirmed
  - 30min 7-hub run: zero queue drops at 25% duty cycle

## [1.0.0] - 2026-02-21

### Initial release

- Passive/active BLE scanning for Pybricks hub broadcasts on Pico 2W
- Decodes all Pybricks value types: int, float, bool, str, bytes, list, tuple
- Channel 255 fix: channel parsed as uint8 not uint16
- Hub registry with letter tags and ANSI colors per hub
- Automatic hub name capture from advertisements and scan responses
- Two-stage name cache: name arriving before first packet is held and
  promoted on first manufacturer packet (prevents non-Pybricks devices
  entering the registry)
- RSSI exponential moving average with human-readable signal labels
- Deduplication: only prints when value changes
- IRQ-safe deque queue with O(1) popleft
- Preventive scan restart (PREVENTIVE_RESTART_EVENTS) and watchdog
- Intentional restart flag prevents spurious "scan stopped unexpectedly"
- Final statistics summary on exit including queue drop detection
- Dark and light terminal color themes
