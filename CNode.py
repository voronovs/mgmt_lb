from typing import cast

import CMGMT
import CDrone2
from CMavlinkGenericConnection import *


class CNode(ABC):
    _mgmt:                  CMGMT.CMGMT
    _connection_string:     str
    _alias:                 str
    _subscribed_messages:   List[int]
    _blacklisted_messages:  List[int]
    _connection:            CMavlinkUDPConnection2

    def __init__(self,
                 alias: str,
                 subscribed_messages: List[int], blacklisted_messages: List[int],
                 connection_string: str,
                 mgmt: CMGMT.CMGMT):
        self._mgmt = mgmt
        self._connection_string = connection_string
        self._alias = alias
        self._subscribed_messages = subscribed_messages
        self._blacklisted_messages = blacklisted_messages
        self._rxCounterDict = {}
        self._txCounterDict = {}
        self._connection = CMavlinkUDPConnection2(self._connection_string)
        self._connection.alias = alias

    @property
    def subscribed_messages(self):
        return self._subscribed_messages

    @property
    def blacklisted_messages(self):
        return self._blacklisted_messages

    @property
    def connection(self):
        return self._connection

    @property
    def alias(self):
        return self._alias

    # abstract class prototype
    @abstractmethod
    def _rx_callback(self, msg: MAVLink_message) -> MAVLink_message:
        return msg

    def send_mavlink_message(self, msg: MAVLink_message) -> None:
        # self.connection.send(msg)
        self._connection.send_buf(msg.get_msgbuf())

    def spin(self, drone: CDrone2.CDrone2, nodes: List['CNode']) -> MAVLink_message:
        if self._connection.is_connected:
            msg = self._connection.spin()
            if msg and self.alias == "Unity":
                print(self._alias, " - ", msg)
            if msg:
                msg = self._rx_callback(msg)
                return msg
        return None

    def get_stats(self, rate_scale: float) -> str:
        return self._connection.get_stats(rate_scale)

    def reset_stats(self) -> None:
        self._connection.reset_stats()


class CLidarNode(CNode):
    def process_message_lidar(self, msg: MAVLink_message) -> MAVLink_message:
        print("Lidar -> (", msg.get_srcSystem(), ")", msg)
        if msg.get_type() == "DEBUG_VECT" and msg.name == "LIDARLIDAR":
            msg = cast(MAVLink_debug_vect_message, msg)
            self._mgmt.process_lidar(msg)
            msg.name = bytes("LIDARLIDAR", "ASCII")
            return msg
        elif msg.get_type() == "DEBUG_VECT" and msg.name == "VELOSTAT1":
            # TODO read fval, max_line_z, packets in last reading, is_valid
            self._mgmt.process_lidar_stats(msg)
            msg.name = bytes("VELOSTAT1", "ASCII")
            return msg

        if msg.get_type() == "NAMED_VALUE_FLOAT" and msg.name == "LIDARPITCH":
            msg = cast(MAVLink_named_value_float_message, msg)
            msg.name = bytes("LIDARPITCH", "ASCII")
            return msg

    def _rx_callback(self, msg: MAVLink_message):
        self.process_message_lidar(msg)
        # msg = self.connection.pack_mavlink_message(msg)
        return msg


class CQGCNode(CNode):
    def _rx_callback(self, msg: MAVLink_message):
        # print("QGC ->", msg)
        msg = self._mgmt.process_mission(msg)
        msg = self.connection.pack_mavlink_message(msg)
        return msg


class CUnityNode(CNode):
    def process_message_unity(self, msg: MAVLink_named_value_int_message):
        # print("2 ->", msg)
        self._mgmt.process_unity(msg)

    def _rx_callback(self, msg: MAVLink_message):
        # print("Unity -> ", msg)
        if msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
            msg = cast(MAVLink_position_target_local_ned_message, msg)
            if msg.type_mask == 0b110111000111:
                return msg
        elif msg.get_type() == "NAMED_VALUE_INT":
            # print("1 ->", msg)
            self.process_message_unity(cast(MAVLink_named_value_int_message, msg))
        return msg
