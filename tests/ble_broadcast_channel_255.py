# Best transfer if NO bluetooth connection is active to the computer

from pybricks.hubs import ThisHub
from pybricks.parameters import Color
from pybricks.tools import wait, StopWatch

BROADCAST_INTERVAL = 500

hub = ThisHub(broadcast_channel=255)
watch = StopWatch()
XMIT_DATA_LIST = [1, 2028, "String3", [4,8,12,16], ('5 tupleA', '5 tupleB')]

counter = 1
while True:
    for xmit_data in XMIT_DATA_LIST:
        watch.reset()
        hub.ble.broadcast(xmit_data)

        # Hub light will be RED if connected to a PC via bluetooth.
        # After disconnect (it takes a few seconds) the light goes to GREEN.
        if hub.system.info()['host_connected_ble']:
            hub.light.on(Color.RED)  # show RED if connected
        else:
            hub.light.on(Color.GREEN)

        wait(BROADCAST_INTERVAL - watch.time())
