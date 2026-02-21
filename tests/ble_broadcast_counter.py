# Best transfer if NO bluetooth connection is active to the computer

broadcast_interval = 100

from pybricks.hubs import ThisHub
from pybricks.tools import wait, StopWatch
hub = ThisHub(broadcast_channel=5)
watch = StopWatch()
print('ble_broadcast_counter')
# send a simple counter. The observer can start at any point and know what the next value should be.
counter = 1
while True:
    try:
        was = counter
        counter += 1
    except:
        counter = 1
    watch.reset()
    hub.ble.broadcast(counter)
    wait(broadcast_interval - watch.time())
