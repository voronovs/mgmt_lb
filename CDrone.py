from typing import cast
from pymavlink.mavutil import mavlink
from CMavlinkUDPConnection import *
from nav_static import *
from CMavlinkGenericConnection import *
import math


class CDrone:
    global_pos: CCoordinate
    _connection: CMavlinkUDPConnection

    def __init__(self, connection):
        self._connection = connection
        self.tsUnity = 0
        self.tsNav = 0
        self.TIMEOUT_SEC = 0.5
        self.srcSystem = 1
        self.pointY = 0
        self.pointZ = 0
        self.normUp = 0
        self.normLeft = 0
        self.normRad = 0
        self.codeState = 5
        self.prevCode = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.roll = 0
        self.pylons_buffer = []
        self.home_buffer = 0
        self.mission_count = 0
        self.is_armed = False
        self.current_mission = None
        self.offset_alt = 3.0
        self.global_pos = CCoordinate(0, 0, 0)
        self.is_taking_off = False
        self.do_auto_land = False
        self.full_land = False
        self.offset_alt_autoland = 0.0
        self.global_height_autoland = 0.0

    def get_stats(self, rate_scale: float) -> str:
        return self._connection.get_stats(rate_scale)

    def reset_stats(self):
        self._connection.reset_stats()

    @property
    def connection(self):
        return self._connection

    def generate_send_message_gs_heartbeat(self):
        self._connection.tx_increment(self._connection.conn)
        # TODO Change to GS ID ?
        self._connection.master.mav.heartbeat_send(
            type=mavlink.MAV_TYPE_GCS,
            autopilot=mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=mavlink.MAV_MODE_MANUAL_ARMED,
            custom_mode=0,
            system_status=mavlink.MAV_STATE_ACTIVE,
        )

    def generate_send_message_debug_vect(self, name, x, y, z):
        self._connection.tx_increment(self._connection.conn)
        self._connection.master.mav.debug_vect_send(name, time.time() * 1e6, x, y, z)

    def generate_send_message_setpoint(self, _x, _y, _z, _yaw):
        self._connection.tx_increment(self._connection.conn)
        self._connection.master.mav.set_position_target_local_ned_send(
            0,
            self.srcSystem,  # target_system
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

    def generate_send_global_setpoint_raw(self, target_lat_int, target_lon_int, alt_asl):
        self._connection.tx_increment(self._connection.conn)
        # print(target_lat_int, target_lon_int, alt_asl)
        # print("Sending to: ", self.connection.conn)
        self._connection.master.mav.set_position_target_global_int_send(
            0,
            self.srcSystem,  # target_system
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

    def generate_send_global_setpoint(self, coordinate: CCoordinate):
        lat, lon, alt = coordinate.get_position_tuple()
        self.generate_send_global_setpoint_raw(lat, lon, alt)

    def generate_send_heading(self, heading: float):
        # print(heading)
        self._connection.tx_increment(self._connection.conn)
        self._connection.master.mav.command_long_send(
            0, 0,  # target system, target component
            mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            0,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used

    def send_mavlink_message(self, msg: MAVLink_message):
        # self.connection.send(msg)
        self._connection.send_buf(msg.get_msgbuf())

    def spin(self, _sta_connections: List[CMavlinkUDPConnection]) -> None:
        msg = self._connection.spin([], [self], _sta_connections)
        if isinstance(msg, int) or (msg is None):
            return
        else:
            self._connection.master.mav.srcSystem = msg.get_srcSystem()
            self._connection.master.mav.srcComponent = msg.get_srcComponent()
            # self.srcSystem = msg.get_srcSystem()
            if msg.get_type() == "GLOBAL_POSITION_INT":
                self.lat = msg.lat
                self.lon = msg.lon
                self.alt = msg.alt
            # for _sta in _sta_connections:
            #     try:
            #         # return
            #         _sta.send_buf(msg.get_msgbuf())
            #     except struct.error as e:
            #         return
            # TODO
            #  Try https://github.com/peterbarker/dronekit-python/blob/
            #  eef45b0cb3e251e6bb4cab56060f91856cd54a02/examples/multivehicle/mavlink_hub.py#L84
            #  sta.sendBuf(msg.get_msgbuf())

    def process_message_heartbeat(self, msg: MAVLink_heartbeat_message):
        if msg.get_srcSystem() == self.srcSystem:
            self.is_armed = (msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0
            if not self.is_armed:
                self.do_auto_land = False
                self.full_land = False

    def process_message_unity(self, msg: MAVLink_message, src_connection: CMavlinkUDPConnection):
        if msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
            msg = cast(MAVLink_set_position_target_local_ned_message, msg)
            if msg.type_mask == 0b110111000111:
                print(msg)
                self.send_mavlink_message(msg)
            return

        if msg.get_type() == "NAMED_VALUE_INT":
            msg = cast(MAVLink_named_value_int_message, msg)
            print(msg)
            if msg.name == "SETNEWSPAN":
                self.current_mission.set_span(msg.value)
                # send packet to Unity from drone system, otherwise Unity drops the packet
                src_connection.generate_send_named_value_int(
                    self, bytearray("SETNEWSPAN", "ASCII"), self.current_mission.get_span())

            if msg.name == "UNITYUNITY":
                if msg.value == 101:
                    msg = src_connection.master.mav.command_long_encode(
                        self.srcSystem,  # target_system
                        1,  # uint8_t target_component
                        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,  # confirmation
                        1.0,  # float param1: doArm
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # float param2-7
                    )
                    msg.pack(src_connection.master.mav)
                    self.send_mavlink_message(msg)

                elif msg.value == 100:
                    msg = src_connection.master.mav.command_long_encode(
                        self.srcSystem,  # target_system
                        1,  # uint8_t target_component
                        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,  # confirmation
                        0.0,  # float param1: doArm
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # float param2-7
                    )
                    msg.pack(src_connection.master.mav)
                    self.send_mavlink_message(msg)

                elif msg.value == 111:
                    self.is_taking_off = True
                    self.do_auto_land = False
                    self.full_land = False
                    msg = src_connection.master.mav.command_long_encode(
                        self.srcSystem,  # target_system
                        1,  # uint8_t target_component
                        mavlink.MAV_CMD_NAV_TAKEOFF,
                        0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        15.0
                    )
                    msg.pack(src_connection.master.mav)
                    self.send_mavlink_message(msg)

                elif msg.value == 110:
                    msg = src_connection.master.mav.command_long_encode(
                        self.srcSystem,  # target_system
                        1,  # uint8_t target_component
                        mavlink.MAV_CMD_NAV_LAND,
                        0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0
                    )
                    msg.pack(src_connection.master.mav)
                    self.send_mavlink_message(msg)

                elif msg.value == 120:
                    mmap = self._connection.master.mode_mapping()
                    self._connection.master.set_mode(mmap["GUIDED"])

                elif msg.value == 130:
                    # Go home
                    self.generate_send_global_setpoint(self.current_mission.get_home_coordinate_3d())
                    self.generate_send_heading(degrees(self.current_mission.get_current_heading()))

                elif msg.value == 131:
                    self.current_mission.advance_span()
                    src_connection.generate_send_named_value_int(
                        self, bytearray("SETNEWSPAN", "ASCII"), self.current_mission.get_span())

                elif msg.value == 132:
                    # GoToLand
                    self.is_taking_off = False
                    self.do_auto_land = False
                    self.full_land = False
                    self.offset_alt = 6.0
                    print("OFFSET_ALT: ", self.offset_alt)
                    setpoint_pos_corrected = \
                        self.current_mission.get_setpoint_position_corrected().offset_alt(self.offset_alt)
                    setpoint_yaw_corrected = self.current_mission.get_setpoint_yaw_corrected()
                    self.generate_send_global_setpoint(setpoint_pos_corrected)
                    self.generate_send_heading(degrees(setpoint_yaw_corrected))

                elif msg.value == 133:
                    # GoToLand Low
                    self.is_taking_off = False
                    self.do_auto_land = False
                    self.full_land = False
                    self.offset_alt = 4.0
                    print("OFFSET_ALT: ", self.offset_alt)
                    setpoint_pos_corrected = \
                        self.current_mission.get_setpoint_position_corrected().offset_alt(self.offset_alt)
                    setpoint_yaw_corrected = self.current_mission.get_setpoint_yaw_corrected()
                    self.generate_send_global_setpoint(setpoint_pos_corrected)
                    self.generate_send_heading(degrees(setpoint_yaw_corrected))

                elif msg.value == 134:
                    self.current_mission.retreat_span()
                    src_connection.generate_send_named_value_int(
                        self, bytearray("SETNEWSPAN", "ASCII"), self.current_mission.get_span())

                elif msg.value == 151:
                    # GoToLand Up
                    self.is_taking_off = False
                    self.do_auto_land = False
                    self.full_land = False
                    self.offset_alt = self.offset_alt + 0.25
                    print("OFFSET_ALT: ", self.offset_alt)
                    setpoint_pos_corrected = self.current_mission.get_setpoint_position_corrected().offset_alt(
                        self.offset_alt)
                    setpoint_yaw_corrected = self.current_mission.get_setpoint_yaw_corrected()
                    self.generate_send_global_setpoint(setpoint_pos_corrected)
                    self.generate_send_heading(degrees(setpoint_yaw_corrected))

                elif msg.value == 152:
                    # GoToLand Down
                    self.is_taking_off = False
                    self.do_auto_land = False
                    self.full_land = False
                    self.offset_alt = self.offset_alt - 0.25
                    print("OFFSET_ALT: ", self.offset_alt)
                    setpoint_pos_corrected = \
                        self.current_mission.get_setpoint_position_corrected().offset_alt(self.offset_alt)
                    setpoint_yaw_corrected = self.current_mission.get_setpoint_yaw_corrected()
                    self.generate_send_global_setpoint(setpoint_pos_corrected)
                    self.generate_send_heading(degrees(setpoint_yaw_corrected))

                elif msg.value == 161:
                    # Auto Land
                    if not self.is_taking_off:
                        print(">>>>>autoland")
                        self.do_auto_land = True
                        self.full_land = False
                        self.offset_alt_autoland = 0.0
                        lat0, lon0, alt0 = self.global_pos.get_position_tuple()
                        self.global_height_autoland = alt0

                elif msg.value == 162:
                    # GoToLand High
                    self.is_taking_off = False
                    self.do_auto_land = False
                    self.full_land = False
                    self.offset_alt = 10.0
                    print("OFFSET_ALT: ", self.offset_alt)
                    setpoint_pos_corrected = \
                        self.current_mission.get_setpoint_position_corrected().offset_alt(self.offset_alt)
                    setpoint_yaw_corrected = self.current_mission.get_setpoint_yaw_corrected()
                    self.generate_send_global_setpoint(setpoint_pos_corrected)
                    self.generate_send_heading(degrees(setpoint_yaw_corrected))

                elif msg.value == 163:
                    self.is_taking_off = True
                    self.do_auto_land = False
                    self.full_land = False
                    msg = src_connection.master.mav.command_long_encode(
                        self.srcSystem,  # target_system
                        1,  # uint8_t target_component
                        mavlink.MAV_CMD_NAV_TAKEOFF,
                        0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        7.0
                    )
                    msg.pack(src_connection.master.mav)
                    self.send_mavlink_message(msg)

    def process_message_lidar(self,
                              msg: MAVLink_message,
                              sta: CMavlinkUDPConnection,
                              sta_connections: List[CMavlinkUDPConnection]):
        msg = cast(MAVLink_debug_vect_message, msg)
        if msg.get_type() == "DEBUG_VECT" and msg.name == "LIDARLIDAR":
            print(msg)
            # # Correct lidar reading for roll
            # up = msg.z
            # right = msg.y
            # dist = sqrt(up*up + right*right)

            print("ARMED = ", self.is_armed)
            if self.is_armed and self.current_mission is not None:
                self.current_mission.set_lidar_position(msg.y, -msg.z, msg.x)
                self.current_mission.calculate_correction_pos_yaw()

                if not self.is_taking_off and (abs(msg.z) < 4.5) and (self.current_mission.is_land_proximity()):
                    # if True:
                    print("AUTOLAND IS ", self.do_auto_land)
                    print("FULL_LAND IS", self.full_land)
                    # decr = 0.0
                    if (abs(msg.y) < 0.2) and self.do_auto_land:
                        # decr = -0.1
                        print("DESCENDING")
                        self.offset_alt = self.offset_alt - 0.05
                        if (abs(msg.z) < 0.35) and (abs(msg.y) < 0.1) and (not self.full_land):
                            self.full_land = True
                            msg = self._connection.master.mav.command_long_encode(
                                self.srcSystem,  # target_system
                                1,  # uint8_t target_component
                                mavlink.MAV_CMD_NAV_LAND,
                                0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0
                            )
                            msg.pack(self._connection.master.mav)
                            self.send_mavlink_message(msg)
                    if not self.full_land:
                        print("OFFSET_ALT: ", self.offset_alt)
                        setpoint_pos_corrected = self.current_mission.get_setpoint_position_corrected().offset_alt(
                            self.offset_alt)
                        # setpoint_pos_corrected = self.global_pos.offset_alt(decr)
                        setpoint_yaw_corrected = self.current_mission.get_setpoint_yaw_corrected()
                        self.generate_send_global_setpoint(setpoint_pos_corrected)
                        self.generate_send_heading(degrees(setpoint_yaw_corrected))
            msg.name = bytes("LIDARLIDAR", "ASCII")
            buf = msg.pack(self._connection.master.mav)
            for _sta in sta_connections:
                if not sta == _sta:
                    _sta.send_buf(buf)
            return

        if msg.get_type() == "NAMED_VALUE_FLOAT" and msg.name == "LIDARPITCH":
            for _sta in sta_connections:
                if not sta == _sta:
                    _sta.send_buf(msg.get_msgbuf())
            return

    def process_message_mission(self,
                                msg: MAVLink_message,
                                src_connection: CMavlinkUDPConnection,
                                sta_connections: List[CMavlinkUDPConnection]):
        if ((msg.get_type() == "MISSION_COUNT") or (msg.get_type() == "MISSION_ITEM_INT") or (
                msg.get_type() == "MISSION_COUNT")):
            print(msg)
            if msg.get_type() == "MISSION_COUNT" and msg.count > 0:
                msg = cast(MAVLink_mission_count_message, msg)
                self.mission_count = msg.count
                self.pylons_buffer = []
                print(self.mission_count)
            if msg.get_type() == "MISSION_ITEM_INT":
                msg = cast(MAVLink_mission_item_int_message, msg)
                if msg.seq == 0:
                    self.home_buffer = CCoordinate(msg.x, msg.y, msg.z + 18.0)
                    print(self.home_buffer.get_position_tuple())
                elif msg.seq > 1:
                    if msg.frame == 3:
                        msg.z = msg.z + self.global_pos.alt
                        msg.frame = 0
                    print(msg)
                    self.pylons_buffer.append(CPylon(CCoordinate(msg.x, msg.y, msg.z)))
                    self.mission_count -= 1
                    print(self.mission_count)
                    if self.mission_count == 2:
                        self.current_mission = CSurveyMission(
                            home_location=self.home_buffer,
                            pylon_list=self.pylons_buffer)
                        print(self.current_mission.spans)
            msg.pack(src_connection.master.mav)
            self.send_mavlink_message(msg)
            for _sta in sta_connections:
                if not _sta == src_connection:
                    print(_sta.conn)
                    _sta.send_buf(msg.get_msgbuf())

    def process_message_pos_att(self, msg: MAVLink_message, sta_connections: List[CMavlinkUDPConnection]):
        if msg.get_type() == "ATTITUDE":
            msg = cast(MAVLink_attitude_message, msg)
            self.roll = msg.roll
            yaw = msg.yaw
            if self.current_mission is not None:
                self.current_mission.set_drone_yaw(yaw)
                yaw_corrected = self.current_mission.get_drone_yaw_corrected()
                msg.yaw = yaw_corrected
            # TODO use encode?
            buf = msg.pack(self._connection.master.mav)
            for _sta in sta_connections:
                _sta.send_buf(buf)
            return

        if msg.get_type() == "GLOBAL_POSITION_INT":
            # print(">>>", msg)
            self.global_pos = CCoordinate(msg.lat, msg.lon, msg.alt * 1e-3)
            if self.current_mission is not None:
                self.current_mission.set_drone_position_global_int(self.global_pos)
                pos_corrected = self.current_mission.get_drone_position_corrected()
                lat1, lon1, alt1 = pos_corrected.get_position_tuple()
                msg.lat = lat1
                msg.lon = lon1
                msg.alt = ceil(alt1 * 1e3)
            # # AUTOLAND PROTO
            # if self.do_auto_land:
            #     print("DESCENDING")
            #     self.offset_alt = self.offset_alt - 0.01
            #     # offset_alt = 0
            #     print(self.offset_alt)
            #     setpoint_pos_corrected = CCoordinate(msg.lat, msg.lon, self.global_height_autoland + self.offset_alt)
            #     print("<<<", self.global_pos.get_position_tuple())
            #     print(">>>", setpoint_pos_corrected.get_position_tuple())
            #     diff = setpoint_pos_corrected - self.global_pos
            #     print("---", diff.get_position_tuple())
            #     self.generate_send_global_setpoint(setpoint_pos_corrected)
            # # END AUTOLAND PROTO
            # msg_out = self.master.mav.global_position_int_encode(
            #     msg.time_boot_ms,
            #     lat1, lon1,
            #     ceil(alt1 * 1e3), msg.relative_alt,
            #     msg.vx, msg.vy, msg.vz,
            #     msg.hdg
            # )
            # print("<<<", msg)

            buf = msg.pack(self._connection.master.mav)
            for _sta in sta_connections:
                _sta.send_buf(buf)
            return
