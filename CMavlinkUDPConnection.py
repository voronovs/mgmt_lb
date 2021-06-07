from typing import Dict

from pymavlink import mavutil
import logging
from nav_static import *
import math

from pymavlink.dialects.v20.common import *


class CMavlinkUDPConnection:
    def __init__(self, conn, _txCounterDict, _rxCounterDict):
        self.conn = conn
        self.REOPEN_TIMEOUT = 1.0
        self.reopenTS = time.time() - 1
        self.isConnected = False
        self.master = None
        self.open()
        self._txCounterDict = _txCounterDict
        self._rxCounterDict = _rxCounterDict
        self._rxCounter = 0
        self._txCounter = 0

    def tx_increment(self, conn_string):
        self._txCounterDict[conn_string] = self._txCounterDict[conn_string] + 1

    def get_stats(self, rate_scale: float) -> str:
        return 'RX/TX {0} ({1} / {2}) \t' \
            .format(
                self.conn,
                math.ceil(self._rxCounter * rate_scale),
                math.ceil(self._txCounter * rate_scale))

    def reset_stats(self):
        self._rxCounter = 0
        self._txCounter = 0
        self._rxCounterDict[self.conn] = 0
        self._txCounterDict[self.conn] = 0

    def open(self):
        if self.reopenTS < time.time():
            self.reopenTS = time.time() + self.REOPEN_TIMEOUT
            try:
                self.master = mavutil.mavlink_connection(self.conn, dialect='common')
                self.isConnected = True
            except Exception as exc:
                logging.error('Error (%s) opening port: %s', exc, self.conn)
                print('Error ({0}) opening port: {1}'.format(exc, self.conn))
                self.isConnected = False

    def send_buf(self, msgBuf, doPrint=False):
        self._txCounterDict[self.conn] = self._txCounterDict[self.conn] + 1
        self._txCounter += 1
        if doPrint:
            print(msgBuf)
        self.master.write(msgBuf)

    def send(self, msg, do_print=False):
        self._txCounterDict[self.conn] = self._txCounterDict[self.conn] + 1
        self._txCounter += 1
        self.master.mav.send(msg)
        if do_print:
            print(msg)

    def pack_mavlink_message(self, msg: MAVLink_message):
        msg.pack(self.master.mav)
        return msg

    def generate_send_debug_vect(self, drone, name, x, y, z):
        msg = self.master.mav.debug_vect_encode(
            name, 0, x, y, z)
        # need to send from drone system, otherwise Unity drops the packet silently
        buf = msg.pack(drone.connection.master.mav)
        self.send_buf(buf)

    def generate_send_named_value_int(self, drone, name, val):
        msg = self.master.mav.named_value_int_encode(
            0, name, val)
        # need to send from drone system, otherwise Unity drops the packet silently
        buf = msg.pack(drone.connection.master.mav)
        self.send_buf(buf)

    def spin(self, _drone_dict: Dict = None,
             _drone_list: List = None,
             _sta_connections: List = None):
        if _drone_list is None:
            _drone_list = []
        if _drone_dict is None:
            _drone_dict = {}
        if _sta_connections is None:
            _sta_connections = []

        try:
            msg: MAVLink_message = self.master.recv_match(blocking=False)

            if msg:
                if msg.get_type() == "BAD_DATA":
                    pass
                else:
                    self._rxCounterDict[self.conn] = self._rxCounterDict[self.conn] + 1
                    self._rxCounter += 1
                    if (msg.get_type() == "DEBUG_VECT" and msg.name == "LIDARLIDAR") \
                            or (msg.get_type() == "NAMED_VALUE_FLOAT" and msg.name == "LIDARPITCH"):
                        [d.process_message_lidar(msg, self, _sta_connections) for d in _drone_list]
                        return None

                    if msg.get_type() == "NAMED_VALUE_INT":
                        if (msg.name == "UNITYUNITY") or (msg.name == "SETNEWSPAN") \
                                or ((msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED") and (
                                msg.type_mask == 0b110111000111)):
                            [d.process_message_unity(msg, self) for d in _drone_list]
                        return None

                    if (msg.get_type() == "ATTITUDE") or (msg.get_type() == "GLOBAL_POSITION_INT"):
                        [d.process_message_pos_att(msg, _sta_connections) for d in _drone_list]
                        return None

                    if msg.get_type() == "HEARTBEAT":
                        if len(_drone_dict) == 0:
                            [d.process_message_heartbeat(msg) for d in _drone_list]

                    if hasattr(msg, 'target_system'):
                        if (
                                (msg.get_type() == "MISSION_COUNT")
                                or (msg.get_type() == "MISSION_ITEM_INT")
                                or (msg.get_type() == "MISSION_COUNT")):

                            if msg.target_system in _drone_dict:
                                _drone_dict[msg.target_system].process_message_mission(msg, self, _sta_connections)
                            return None

                        if msg.target_system in _drone_dict:
                            if not (self == _drone_dict[msg.target_system].connection):
                                _drone_dict[msg.target_system].send_mavlink_message(msg)
                    else:
                        # if doesn't have target_id, then drop the packet?
                        # No, send to each drone
                        for d in _drone_list:
                            if not (self == d.connection):
                                d.send_mavlink_message(msg)

                for _sta in _sta_connections:
                    if not (self == _sta):
                        _sta.send_buf(msg.get_msgbuf())

                return msg
        except Exception as exc:
            self.open()
            print(exc)
            logging.error('Error (%s)', exc)
