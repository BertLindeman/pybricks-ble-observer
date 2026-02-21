
from pybricks.hubs import ThisHub
from pybricks.tools import wait, StopWatch

# all_ch = [x for x in range(256)]

while True:
    for ch in range(256):
        hub = None
        hub = ThisHub(broadcast_channel=ch)
        wait(10)
        hub.ble.broadcast(f"ch_{ch}")
        # print(f"ch_{ch}")
        wait(100)
