from __future__ import print_function

import sys

from CMavlinkSerialConnection import *
from CMavlinkUDPConnection import *
from CDrone import *
from nav_static import *

logging.basicConfig(level=logging.DEBUG, filename='app.log', filemode='w',
                    format='[%(asctime)s][%(levelname)s] %(message)s', datefmt='[%d-%b-%y %H:%M:%S]')
# logging.basicConfig(
#     level=logging.INFO,
#     format="%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
#     handlers=[
#         logging.FileHandler("{0}/{1}.log".format(logPath, fileName)),
#         logging.StreamHandler()
#     ])
logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))
logging.debug('App starting')

# baseStations = ['udpout:192.168.2.29:14550']
#baseStations = ['udpout:192.168.2.21:14551']  # , 'udpout:127.0.0.1:14560', 'udpout:127.0.0.1:14552', 'udpin:0.0.0.0:14553']
#baseStations = ['udpout:192.168.88.95:14551', 'udpout:192.168.88.90:14550', 'udpout:127.0.0.1:14552']
baseStations = ['udpout:127.0.0.1:14552', 'udpout:192.168.88.97:14551', 'udpin:0.0.0.0:14554']
udpDrones = ['udpin:0.0.0.0:14550']  # serial port list
serialPorts = []  # ['/dev/tty.SLAB_USBtoUART'] # serial port list
SERIAL_BAUD = 460800

droneDict = {}  # holder dict for drones: id -> instance
droneList = []
baseStationConnections = []  # holder list for base stations
serialConnections = []  # holder for drone serial connections
rxCounterDict = {}
txCounterDict = {}
connList = []

doExit = False


def wait_heartbeat(conn_wait):
    """wait for a heartbeat so we know the target system IDs"""
    print("Waiting for heartbeat on conn {0}".format(conn_wait))
    conn_wait.master.wait_heartbeat()
    print("Heartbeat (system %u component %u)" % (conn_wait.master.target_system, conn_wait.master.target_component))


for conn in serialPorts:
    droneConn = CMavlinkSerialConnection(conn, SERIAL_BAUD, txCounterDict, rxCounterDict)
    drone = CDrone(droneConn)
    while not droneConn.isConnected:
        droneConn.open()
        time.sleep(0.1)
    wait_heartbeat(droneConn)
    droneList.append(drone)
    rxCounterDict[conn] = 0
    txCounterDict[conn] = 0
    connList.append(conn)

for conn in udpDrones:
    droneConn = CMavlinkUDPConnection(conn, txCounterDict, rxCounterDict)
    drone = CDrone(droneConn)
    while not droneConn.isConnected:
        droneConn.open()
        time.sleep(0.1)
    wait_heartbeat(droneConn)
    droneList.append(drone)
    rxCounterDict[conn] = 0
    txCounterDict[conn] = 0
    connList.append(conn)

for conn in baseStations:
    staConn = CMavlinkUDPConnection(conn, txCounterDict, rxCounterDict)
    baseStationConnections.append(staConn)
    rxCounterDict[conn] = 0
    txCounterDict[conn] = 0
    connList.append(conn)

bootTime = time.time()
cycleTime = bootTime
hbTime = bootTime
spins = 0

logging.debug("All ports open")

while not doExit:
    ct = time.time()

    for sta in baseStationConnections:
        sta.spin(droneDict, droneList, baseStationConnections)

    for drone in droneList:
        drone.spin(baseStationConnections)
        if (drone.srcSystem > 0) and (drone.srcSystem not in droneDict):
            droneDict[drone.srcSystem] = drone

    spins = spins + 1
    time.sleep(0.0001)

    if ct - hbTime > 0.5:
        # TODO
        #  Need this to identify us as GS, e.g. receive statustext
        for drone in droneList:
            if drone.srcSystem > 0:
                drone.generate_send_message_gs_heartbeat()
        hbTime = ct

    if ct - cycleTime > 2.0:
        print("Spins: ", spins, end='\t')
        for conn in connList:
            print('RX/TX {0} ({1} / {2}) \t'.format(conn, rxCounterDict[conn]/2, txCounterDict[conn]/2), end='')
            rxCounterDict[conn] = 0
            txCounterDict[conn] = 0
        print(end='\n')
        spins = 0
        rxUnity = 0
        rxNav = 0
        cycleTime = ct
