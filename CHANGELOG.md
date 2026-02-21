# Changelog

All notable changes to this project will be documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

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
