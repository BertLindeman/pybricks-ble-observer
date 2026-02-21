# Best transfer if NO bluetooth connection is active to the computer

broadcast_interval = 250

from pybricks.hubs import ThisHub
from pybricks.tools import wait, StopWatch
hub = ThisHub(broadcast_channel=255)
watch = StopWatch()
XMIT_DATA_LIST = [1, 2028, "String3", [4,8,12,16], ('5 tupleA', '5 tupleB')]

counter = 1
while True:
    for xmit_data in XMIT_DATA_LIST:
        watch.reset()
        hub.ble.broadcast(xmit_data)
        wait(broadcast_interval - watch.time())
