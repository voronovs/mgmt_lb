from typing import cast

import CMGMT
from CMavlinkGenericConnection import *
import CNode


class CDrone2:
    _mgmt: 'CMGMT.CMGMT'
    _connection: CMavlinkGenericConnection

    def __init__(self, alias: str, connection: CMavlinkGenericConnection, mgmt: 'CMGMT.CMGMT'):
        self._connection = connection
        self._connection.alias = alias
        self._mgmt = mgmt
        self._is_armed = False
        self._system_id = 1
        self._lat = 0
        self._lon = 0
        self._alt = 0.0
        self._roll = 0.0
        self._global_pos = CCoordinate(0, 0, 0.0)
        self._alias = alias
        self.WHEELS_DRIVER_PORT = 9
        self.WHEELS_DRIVER_FORWARD_PWM = 982
        self.WHEELS_DRIVER_BACK_PWM = 2006
        self.PANTOGRAF_SERVO_PORT = 8
        self.PANTOGRAF_SERVO_CLOSE_PWM = 1085
        self.PANTOGRAF_SERVO_OPEN_PWM = 2006
        self._distance_to_next_tower = 0
        pass

    def spin(self) -> MAVLink_message:
        if self._connection.is_connected:
            msg = self._connection.spin()
            # print("2. ", self._connection._connection_string, " - ", msg)
            if msg is not None:
                msg = self._rx_callback(msg)
                # self._forward_mavlink(msg, self._mgmt.nodes)
                return msg
        return None

    def _rx_callback(self, msg: MAVLink_message): 
        return self.process_messages(msg)

    def send_buf(self, msg_buf, doPrint=False):
        self._connection.send_buf(msg_buf, doPrint)

    def send(self, msg, do_print=False):
        self._connection.send(msg, do_print)

    def generate_send_message_gs_heartbeat(self):
        # TODO Change to GS ID ?
        msg = self._connection.master.mav.heartbeat_encode(
            type=mavlink.MAV_TYPE_GCS,
            autopilot=mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=mavlink.MAV_MODE_MANUAL_ARMED,
            custom_mode=0,
            system_status=mavlink.MAV_STATE_ACTIVE,
        )
        msg.pack(self._connection.master.mav)
        self.send(msg)

    def arm(self, source_mav: MAVLink):
        msg = self._connection.master.mav.command_long_encode(
            self._system_id,  # target_system
            1,  # uint8_t target_component
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1.0,  # float param1: doArm
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # float param2-7
        )
        msg.pack(source_mav)
        self.send(msg)

    def pantorgraf_servo_close(self, source_mav: MAVLink):
        msg = self._connection.master.mav.command_long_encode(self._system_id, 1,
                                   mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                   self.PANTOGRAF_SERVO_PORT, self.PANTOGRAF_SERVO_CLOSE_PWM,
                                   0, 0, 0, 0, 0)
        msg.pack(source_mav)
        self.send(msg)

    def pantorgraf_servo_open(self, source_mav: MAVLink):
        msg = self._connection.master.mav.command_long_encode(self._system_id, 1,
                                   mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                   self.PANTOGRAF_SERVO_PORT, self.PANTOGRAF_SERVO_OPEN_PWM,
                                   0, 0, 0, 0, 0)
        msg.pack(source_mav)
        self.send(msg)

    def wheels_forward(self, source_mav: MAVLink, speed):
        msg = self._connection.master.mav.command_long_encode(self._system_id, 1,
                                   mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                   self.WHEELS_DRIVER_PORT, speed,
                                   0, 0, 0, 0, 0)
        msg.pack(source_mav)
        self.send(msg)

    def wheels_back(self, source_mav: MAVLink):
        msg = self._connection.master.mav.command_long_encode(self._system_id, 1,
                                   mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                   self.WHEELS_DRIVER_PORT, self.WHEELS_DRIVER_BACK_PWM,
                                   0, 0, 0, 0, 0)
        msg.pack(source_mav)
        self.send(msg)

    def disarm(self, source_mav: MAVLink):
        msg = self._connection.master.mav.command_long_encode(
            self._system_id,  # target_system
            1,  # uint8_t target_component
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0.0,  # float param1: doArm
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # float param2-7
        )
        msg.pack(source_mav)
        self.send(msg)

    def takeoff(self, height, source_mav: MAVLink):
        msg = self._connection.master.mav.command_long_encode(
            self._system_id,  # target_system
            1,  # uint8_t target_component
            mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            height
        )
        msg.pack(source_mav)
        self.send(msg)

    def land(self, source_mav: MAVLink):
        msg = self._connection.master.mav.command_long_encode(
            self._system_id,  # target_system
            1,  # uint8_t target_component
            mavlink.MAV_CMD_NAV_LAND,
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0
        )
        msg.pack(source_mav)
        self.send(msg)

    def generate_send_mgmt_status(self):
        val = self._mgmt.generate_mgmt_status_value()
        msg = self._connection.generate_pack_named_value_int(
            bytearray("MGMT_STATS", "ASCII"), val)
        self._mgmt.broadcast_mavlink_message(msg)

    def generate_send_distance_to_next_tower(self):
        self._distance_to_next_tower = int(self._global_pos.get_2d_accurate_distance_to(self._mgmt.current_mission.get_current_span()._pylon_end.get_coordinate()))
        val = self._distance_to_next_tower
        msg = self._connection.generate_pack_named_value_int(
            bytearray("DIST_NEXTT", "ASCII"), val)
        self._mgmt.broadcast_mavlink_message(msg)

    def generate_send_lidar_status(self):
        val = self._mgmt.generate_lidar_status_value()
        msg = self._connection.generate_pack_named_value_int(
            bytearray("VELO_STATS", "ASCII"), val)
        # print("send lidar status")
        self._mgmt.broadcast_mavlink_message(msg)

    def generate_send_global_setpoint(self, coordinate: CCoordinate):
        lat, lon, alt = coordinate.get_position_tuple()
        msg = self._connection.generate_pack_global_setpoint_raw(lat, lon, alt, self._system_id)
        self._mgmt.broadcast_mavlink_message(msg)

    def generate_send_heading(self, heading: float):
        msg = self._connection.generate_pack_heading_setpoint(heading)
        self._connection.send(msg)

    def set_flight_mode(self, mode: str):
        mmap = self._connection.master.mode_mapping()
        self._connection.master.set_mode(mmap[mode])

    def get_stats(self, rate_scale: float) -> str:
        return self._connection.get_stats(rate_scale)

    def reset_stats(self) -> None:
        self._connection.reset_stats()

    def process_message_heartbeat(self, msg: MAVLink_heartbeat_message):
        self._connection.master.mav.srcSystem = self._system_id
        if msg.get_srcSystem() == self._system_id:
            self._is_armed = (msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0
            if not self._is_armed:
                self._mgmt.auto_land = False
                self._mgmt.full_land = False

    def process_message_pos_att(self, msg: MAVLink_message):
        if msg.get_type() == "ATTITUDE":
            msg = cast(MAVLink_attitude_message, msg)
            self._roll = msg.roll
            yaw = msg.yaw
            if self._mgmt.current_mission is not None:
                self._mgmt.current_mission.set_drone_yaw(yaw)
                yaw_corrected = self._mgmt.current_mission.get_drone_yaw_corrected()
                msg.yaw = yaw_corrected

            msg.pack(self._connection.master.mav)

        if msg.get_type() == "GLOBAL_POSITION_INT":
            msg = cast(MAVLink_global_position_int_message, msg)
            self._lat = msg.lat
            self._lon = msg.lon
            self._alt = msg.alt
            # print(">>>", msg)
            self._global_pos = CCoordinate(msg.lat, msg.lon, msg.alt * 1e-3)
            if self._mgmt.current_mission is not None:
                self._mgmt.current_mission.set_drone_position_global_int(self._global_pos)
                pos_corrected = self._mgmt.current_mission.get_drone_position_corrected()
                lat1, lon1, alt1 = pos_corrected.get_position_tuple()
                msg.lat = lat1
                msg.lon = lon1
                msg.alt = ceil(alt1 * 1e3)

            msg.pack(self._connection.master.mav)

        return msg

    def process_messages(self, msg: MAVLink_message) -> MAVLink_message:
        if msg.get_type() == "HEARTBEAT":
            self.process_message_heartbeat(cast(MAVLink_heartbeat_message, msg))
        elif msg.get_type() == "ATTITUDE" or msg.get_type() == "GLOBAL_POSITION_INT":
            return self.process_message_pos_att(msg)

        return msg

    @property
    def connection(self):
        return self._connection

    @property
    def global_pos(self):
        return self._global_pos

    @property
    def is_armed(self):
        return self._is_armed
