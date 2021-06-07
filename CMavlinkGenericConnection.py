from pymavlink import mavutil
import logging

from pymavlink.mavutil import mavlink

from nav_static import *
import math

from pymavlink.dialects.v20.common import *
from abc import ABC, abstractmethod


class CMavlinkGenericConnection(ABC):

    def __init__(self):
        self._connection_string = 'None'
        self.REOPEN_TIMEOUT = 1.0
        self.reopenTS = time.time() - 1
        self._is_connected = False
        self._rxCounter = 0
        self._txCounter = 0
        self._master = None
        self._is_connected = False
        self._alias = 'noname'

    @property
    def alias(self):
        return self._alias

    @alias.setter
    def alias(self, new_alias: str):
        self._alias = new_alias

    def get_stats(self, rate_scale: float) -> str:
        return '{0}: {1} / {2} \t' \
            .format(
                self._alias,
                math.ceil(self._rxCounter * rate_scale),
                math.ceil(self._txCounter * rate_scale))

    def reset_stats(self):
        self._rxCounter = 0
        self._txCounter = 0

    def open(self):
        if self.reopenTS < time.time():
            self.reopenTS = time.time() + self.REOPEN_TIMEOUT
            try:
                self._open()
                self._is_connected = True
            except Exception as exc:
                logging.error('Error (%s) opening port: %s', exc, self._connection_string)
                print('Error ({0}) opening port: {1}'.format(exc, self._connection_string))
                self._is_connected = False

    @abstractmethod
    def _open(self):
        pass

    def spin(self):
        try:
            msg: MAVLink_message = self._master.recv_match(blocking=False)
            # if msg and self._connection_string == 'udpout:192.168.88.17:14551':
            #     print(self._connection_string, " = ", msg)
            if msg:
                # print(self._connection_string, " - ", msg)
                if msg.get_type() == "BAD_DATA":
                    return None
                else:
                    self._rxCounter += 1
                    return msg
            else:
                return None
        except Exception as exc:
            self.open()
            # print("Exc:  ", exc)
            logging.error('Error (%s): %s', self.alias, exc)
            return None

    def send_buf(self, msg_buf, do_print=False):
        self._txCounter += 1
        if do_print:
            print(msg_buf)
        self._master.write(msg_buf)

    def send(self, msg: MAVLink_message, do_print=False):
        if do_print:
            print(msg)
        self.send_buf(msg.get_msgbuf(), do_print)

    def pack_mavlink_message(self, msg: MAVLink_message):
        if hasattr(msg, 'param_id'):
            msg.param_id = msg.param_id.encode('ASCII')
        msg.pack(self._master.mav)
        return msg

    def generate_pack_message_gs_heartbeat(self) -> MAVLink_heartbeat_message:
        # TODO Change to GS ID ?
        msg = self._master.mav.heartbeat_encode(
            type=mavlink.MAV_TYPE_GCS,
            autopilot=mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=mavlink.MAV_MODE_MANUAL_ARMED,
            custom_mode=0,
            system_status=mavlink.MAV_STATE_ACTIVE,
        )
        self.pack_mavlink_message(msg)
        return msg

    def generate_pack_debug_vect(self, name, x, y, z) -> MAVLink_debug_vect_message:
        msg = self._master.mav.debug_vect_encode(
            name, 0, x, y, z)
        # TODO need to send from drone system, otherwise Unity drops the packet silently
        self.pack_mavlink_message(msg)
        return msg

    def generate_pack_named_value_int(self, name, val) -> MAVLink_named_value_int_message:
        msg = self._master.mav.named_value_int_encode(
            0, name, val)
        # TODO need to send from drone system, otherwise Unity drops the packet silently
        self.pack_mavlink_message(msg)
        return msg

    def generate_pack_message_setpoint(self, _x, _y, _z, _yaw, target_system):
        msg = self._master.mav.set_position_target_local_ned_encode(
            0,
            target_system,  # target_system
            1,  # target_component
            1,  # coordinate_frame = MAV_FRAME_LOCAL_NED
            504,  # type_mask = B111111000
            _x,
            _y,
            _z,
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            _yaw,
            0.0)  # yaw_rate
        self.pack_mavlink_message(msg)
        return msg

    def generate_pack_global_setpoint_raw(self, target_lat_int, target_lon_int, alt_asl, target_system):
        msg = self._master.mav.set_position_target_global_int_encode(
            0,
            target_system,  # target_system
            1,  # target_component
            mavlink.MAV_FRAME_GLOBAL_INT,
            # mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,#MAV_FRAME_GLOBAL_INT,   # coordinate_frame
            0b0000111111111000,  # 0b010111111000,                 # type_mask
            target_lat_int,
            target_lon_int,
            alt_asl,
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0,
            0.0)  # yaw_rate
        self.pack_mavlink_message(msg)
        return msg

    def generate_pack_global_setpoint(self, coordinate: CCoordinate, target_system):
        lat, lon, alt = coordinate.get_position_tuple()
        return self.generate_pack_global_setpoint_raw(lat, lon, alt, target_system)

    def generate_pack_heading_setpoint(self, heading: float):

        if heading < 0.0:
            heading += 360.0
        elif heading > 360.0:
            heading -= 360.0

        print("heading", heading)

        msg = self._master.mav.command_long_encode(
            0, 0,  # target system, target component
            mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            0,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used)
        self.pack_mavlink_message(msg)
        return msg

    @property
    def master(self):
        return self._master

    @property
    def is_connected(self):
        return self._is_connected


class CMavlinkSerialConnection2(CMavlinkGenericConnection):
    def __init__(self, port, baud):
        super().__init__()
        self._port = port
        self._baud = baud
        self._connection_string = '{0}@{1}'.format(port, baud)
        self._open()

    def _open(self):
        self._master = mavutil.mavserial(
            self._port,
            self._baud,
            source_system=255,
            source_component=0,
            autoreconnect=True,
            use_native=False,
            force_connected=False
        )
        # TODO re-open
        self._is_connected = True


class CMavlinkUDPConnection2(CMavlinkGenericConnection):
    def __init__(self, connection_string):
        super().__init__()
        self._connection_string = connection_string
        self._open()

    def _open(self):
        self._master = mavutil.mavlink_connection(self._connection_string, dialect='common')
        # TODO re-open
        self._is_connected = True
