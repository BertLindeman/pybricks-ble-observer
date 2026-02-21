
from pybricks.hubs import ThisHub
from pybricks.tools import wait, StopWatch
from pybricks import version 
from pybricks.parameters import Color 

print(version)

# Dwell on each channel long enough for the observer's 25% duty cycle
# scan window to catch one of the BLE chip's automatic repeat broadcasts.
# 500ms dwell gives ~5 advertising intervals at 100ms — high catch probability.
# Full sweep of 256 channels takes ~128 seconds.
DWELL_MS = 500

while True:
    for ch in range(256):
        hub = ThisHub(broadcast_channel=ch)
        hub.ble.broadcast(f"ch_{ch}")

        # Hub light will be RED if connected to a PC via bluetooth.
        # After disconnect (it takes a few seconds) the light goes to GREEN.
        if hub.system.info()['host_connected_ble']:
            hub.light.on(Color.RED)  # show RED if connected
        else:
            hub.light.on(Color.GREEN)

        wait(DWELL_MS)
