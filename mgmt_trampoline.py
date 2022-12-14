import CNode
import CDrone2
import CMGMT
from CMavlinkGenericConnection import *
import os

if os.name == 'nt':
    from timer_resolution import *

logging.basicConfig(level=logging.DEBUG, filename='app.log', filemode='w',
                    format='[%(asctime)s][%(levelname)s] %(message)s', datefmt='[%d-%b-%y %H:%M:%S]')
logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))
logging.debug('App starting')

mgmt = CMGMT.CMGMT()

node_lidar = CNode.CLidarNode(
    alias='Lidar',
    subscribed_messages=[MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_ATTITUDE, MAVLINK_MSG_ID_DEBUG_VECT],
    blacklisted_messages=[],
    connection_string='udpin:0.0.0.0:14554',
    mgmt=mgmt)

node_qgroundcontrol = CNode.CQGCNode(
    alias='QGC',
    subscribed_messages=[],
    blacklisted_messages=[],
    connection_string='udpout:192.168.88.85:14550',
    mgmt=mgmt)

node_unity = CNode.CUnityNode(
    alias='Unity',
    subscribed_messages=
    [MAVLINK_MSG_ID_HEARTBEAT,
     MAVLINK_MSG_ID_SYS_STATUS,
     MAVLINK_MSG_ID_STATUSTEXT,
     MAVLINK_MSG_ID_RADIO_STATUS,
     MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
     MAVLINK_MSG_ID_ATTITUDE,
     MAVLINK_MSG_ID_NAMED_VALUE_INT,
     MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
     MAVLINK_MSG_ID_DEBUG_VECT,
     MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY,
     MAVLINK_MSG_ID_MISSION_COUNT,
     MAVLINK_MSG_ID_MISSION_ITEM_INT,
     MAVLINK_MSG_ID_MISSION_ITEM,
     MAVLINK_MSG_ID_MISSION_ACK],
    blacklisted_messages=[],
    connection_string='udpout:192.168.88.85:14558',
    mgmt=mgmt)

nodes = [node_unity, node_qgroundcontrol, node_lidar]

rxCounterDict = {}
txCounterDict = {}
# drone_connection = CMavlinkSerialConnection2('/dev/tty.usbmodem142201', 921600)
drone_connection = CMavlinkUDPConnection2('udpin:0.0.0.0:14559')
# pci-0000:00:14.0-usb-0:2:1.0-port0  pci-0000:00:14.0-usb-0:4:1.0-port0
# drone_connection = CMavlinkSerialConnection2('/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0', 500000)
# drone_connection = CMavlinkSerialConnection2('/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0', 500000)
# drone_connection = CMavlinkSerialConnection2('/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0', 500000)
drone = CDrone2.CDrone2('D', drone_connection, mgmt)

mgmt.set_drone(drone)
mgmt.set_nodes(nodes)
mgmt.try_load_mission()

doExit = False

while not drone_connection.is_connected:
    drone_connection.open()
    time.sleep(0.1)

print("Waiting for heartbeat on conn {0}".format(drone_connection))
drone_connection.master.wait_heartbeat()
print("Heartbeat (system %u component %u)"
      % (drone_connection.master.target_system, drone_connection.master.target_component))

bootTime = time.time()
cycleTime = bootTime
hbTime = bootTime
spins = 0

logging.debug("All ports open")

while not doExit:
    ct = time.time()

    mgmt.spin()

    spins = spins + 1
    if os.name == 'nt':
        with timer_resolution(msecs=1):
            time.sleep(0.001)
    else:
        time.sleep(0.001)

    if ct - hbTime > 0.5:
        # TODO
        #  Need this to identify us as GS, e.g. receive statustext
        #  Maybe remove since we are forwarding messages from QGC
        drone.generate_send_message_gs_heartbeat()
        # Also send MGMT status
        drone.generate_send_lidar_status()
        drone.generate_send_mgmt_status()
        drone.generate_send_distance_to_next_tower()

        if drone.is_armed:
            drone.pantorgraf_servo_open(drone._mgmt._mav)
        else:
            drone.pantorgraf_servo_close(drone._mgmt._mav)

        drone._mgmt.move_to_next_tower()

        hbTime = ct

    if ct - cycleTime > 2.0:
        print("Spins: ", spins, end='\t')
        print(mgmt.get_stats(0.5))
        mgmt.reset_stats()
        spins = 0
        rxUnity = 0
        rxNav = 0
        cycleTime = ct
