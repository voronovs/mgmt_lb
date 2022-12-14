import struct
import time
from typing import cast

from pymavlink.dialects.v20.common import MAVLink_debug_vect_message, MAVLink_message, MAVLink_mission_count_message, \
    MAVLink_mission_item_int_message, MAVLink_named_value_int_message, MAVLink, MAV_CMD_NAV_TAKEOFF
from pymavlink.mavutil import mavlink

import CDrone2
import CNode

from nav_static import *

import serial


# Main state machine fro mgmt server
class CMGMT:
    _drone: CDrone2.CDrone2
    _nodes: List['CNode.CNode']

    def __init__(self):
        self._drone = None
        self._nodes = []

        self._pylons_buffer = []
        self._home_buffer = 0
        self._mission_count = 0
        self._current_mission = None
        self._mission_loaded = False

        self._pylon_alt = 30.0
        self._takeoff_alt = 30.0
        self._takeoff_alt_from_wire = 9.0

        self._auto_navigation_mode = 0 # 0-none, 1-LandB and ready for LandC, 2 - LandC

        self.offset_alt = 3.0
        self.is_taking_off = False
        self._auto_land = False
        self._full_land = False
        self._auto_land_available = False

        self._lidar_healthy = False
        self._lidar_streaming = False
        self._lidar_last_fval = 237
        self._lidar_last_fval_raw = 2.0
        self._lidar_max_line_z = 9.7
        self._lidar_distance_vert = 0.0
        self._lidar_distance_hor = 0.0
        self.LIDAR_DIST_HOR_THRESHOLD = 0.1
        self.LIDAR_DIST_VERT_THRESHOLD_FULL_LAND = 0.37
        self.LIDAR_DIST_VERT_THRESHOLD_AUTOLAND = 4.5
        self.LIDAR_TIMEOUT = 1.0
        self.LIDAR_FVAL_THRESHOLD = 0.4
        self._lidar_last_timestamp = 0.0
        self._lidar_healthy = False
        self._lidar_ranging_active = False
        self._lidar_nfound_ok = False
        self._lidar_nfound_near_ok = False
        self._lidar_points_ok = False
        self._lidar_euler_ok = False
        self._lidar_fval_ok = False
        self._lidar_tracking_valid = False
        self._lidar_solution_ok_sending = False
        self._lidar_use_pcap = False

        self.LIDAR_COMPONENT_ID = 57

        self._mav = MAVLink(None, 1, self.LIDAR_COMPONENT_ID)

        self.MISSION_BUFFER_FILENAME = 'mission_buffer.json'

        self._first_approach = True

        self._wheels_move_flag = False
        self._wheels_stop_flag = True
        self.stop_distance_to_tower = 9

    def set_drone(self, drone: 'CDrone2.CDrone2'):
        self._drone = drone

    def set_nodes(self, nodes: List['CNode.CNode']):
        self._nodes = nodes

    @property
    def drone(self):
        return self._drone

    @property
    def nodes(self):
        return self._nodes

    @property
    def auto_land(self):
        return self._auto_land

    @auto_land.setter
    def auto_land(self, do_auto_land: bool):
        self._auto_land = do_auto_land

    @property
    def full_land(self):
        return self._full_land

    @full_land.setter
    def full_land(self, do_full_land: bool):
        self._full_land = do_full_land

    @property
    def lidar_timeout(self):
        return (time.time() - self._lidar_last_timestamp) > self.LIDAR_TIMEOUT

    @property
    def lidar_fval_high(self):
        return self._lidar_last_fval_raw < self.LIDAR_FVAL_THRESHOLD

    def lidar_above_wire(self, dist_threshold):
        return abs(self._lidar_distance_hor) < dist_threshold

    @property
    def lidar_healthy(self):
        self._lidar_healthy = \
            ((not self.lidar_timeout) or self._lidar_use_pcap) \
            and (not self.lidar_fval_high) \
            and self._lidar_tracking_valid
        return self._lidar_healthy

    @property
    def mission_loaded(self):
        return self._mission_loaded

    @property
    def current_mission(self):
        if self.mission_loaded:
            return self._current_mission
        else:
            return CSurveyMission(home_location=None, pylon_list=None)

    @property
    def auto_land_available(self):
        self._auto_land_available = \
            (not self.is_taking_off) \
            and self.lidar_healthy \
            and (abs(self._lidar_distance_vert) < self.LIDAR_DIST_VERT_THRESHOLD_AUTOLAND) \
            and self.lidar_above_wire(0.5) \
            and self._drone.is_armed
        return self._auto_land_available

    def lidar_should_activate(self):
        return (not self._lidar_ranging_active) and (self.drone.global_pos.alt > self._pylon_alt)

    def activate_lidar(self):
        #msg = self.drone.connection.generate_pack_debug_vect(bytearray("CORRECTION", "ASCII"), lat, lon, alt)
        #self.broadcast_mavlink_message(msg)
        pass

    def generate_mgmt_status_value(self):
        val = \
            self.auto_land_available * 0b01000000000000000000000000000000 \
            + self.auto_land * 0b00100000000000000000000000000000 \
            + self.full_land * 0b00010000000000000000000000000000 \
            + self.mission_loaded * 0b00001000000000000000000000000000
        return val

    def generate_lidar_status_value(self):
        fval = (self._lidar_last_fval << 16)
        mlz = min(65535, floor(self._lidar_max_line_z / 160.0 * 65536.0)) & 0x0000ffff

        val = \
            self._lidar_tracking_valid * 0b01000000000000000000000000000000 \
            + self._lidar_healthy * 0b00100000000000000000000000000000 \
            + fval + mlz

        return val

    def _forward_mavlink(self, msg: MAVLink_message, drone: CDrone2.CDrone2, nodes: List['CNode'],
                         src_node=None) -> None:
        if drone:
            drone.send_buf(msg.get_msgbuf())
        for node in nodes:
            if not src_node or node != src_node:
                if (msg.get_msgId() not in node.blacklisted_messages) \
                        and ((msg.get_msgId() in node.subscribed_messages) or (not node.subscribed_messages)):
                    node.connection.send_buf(msg.get_msgbuf())

    def spin(self):
        msg = self._drone.spin()
        if msg:
            self._forward_mavlink(msg, None, self._nodes)
        for node in self._nodes:
            msg = node.spin(self._drone, self._nodes)
            if msg:
                self._forward_mavlink(msg, self._drone, self._nodes, node)

    def get_stats(self, rate_scale: float) -> str:
        s = self._drone.get_stats(rate_scale)
        for node in self._nodes:
            s += ' ' + node.get_stats(rate_scale)
        return s

    def reset_stats(self) -> None:
        self._drone.reset_stats()
        for node in self._nodes:
            node.reset_stats()

    def pack_mavlink_message(self, msg: MAVLink_message):
        msg.pack(self._mav)
        return msg

    def broadcast_mavlink_message(self, msg: MAVLink_message):
        self._drone.send(msg, False)
        for node in self._nodes:
            node.send_mavlink_message(msg)

    def process_lidar(self, msg: MAVLink_debug_vect_message) -> None:
        # print(msg)
        # print("ARMED = ", self.drone._is_armed)
        if self.drone.is_armed and self.mission_loaded:
            self.current_mission.set_lidar_position(msg.y, -msg.z, msg.x)
            self.current_mission.calculate_correction_pos_yaw()

            self._lidar_distance_vert = msg.z
            self._lidar_distance_hor = msg.y
            if \
                    not self.is_taking_off \
                            and (abs(self._lidar_distance_vert) < self.LIDAR_DIST_VERT_THRESHOLD_AUTOLAND) \
                            and (self.current_mission.is_land_proximity()):
                # if True:
                # print("AUTOLAND IS ", self._auto_land)
                # print("FULL_LAND IS", self._full_land)
                if self.lidar_above_wire(0.2) and self.auto_land:
                    # print("DESCENDING")
                    self.offset_alt = self.offset_alt - 0.06
                    if \
                            (abs(self._lidar_distance_vert) < self.LIDAR_DIST_VERT_THRESHOLD_FULL_LAND) \
                                    and self.lidar_above_wire(0.1) \
                                    and (not self.full_land):
                        self.full_land = True
                        self._drone.land(self._mav)
                if not self._full_land:
                    print("OFFSET_ALT: ", self.offset_alt)
                    self.goto_span_land(self.offset_alt)

        if self._auto_navigation_mode == 1:     #LandB
            if abs(self._lidar_distance_vert) < 6.5:
                self._auto_navigation_mode = 2
                # GoToLandC
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = 4.0
                self.goto_span_land(self.offset_alt)
        elif self._auto_navigation_mode == 2:   #LandC
            if abs(self._lidar_distance_vert) < 4.5:
                self._auto_navigation_mode = 0

    def process_lidar_stats(self, msg: MAVLink_debug_vect_message) -> None:
        self._lidar_last_timestamp = time.time()
        fval_raw = msg.x
        self._lidar_last_fval = int(min(255, max(0, 255 - 255 * msg.x / self.LIDAR_FVAL_THRESHOLD)))
        self._lidar_max_line_z = msg.z
        integer_buffer = struct.unpack('I', struct.pack('f', msg.y))[0]
        # see https://github.com/tsuru-robotics/mgmt_lb/issues/7
        # for comments
        self._lidar_streaming = (integer_buffer & 1 << (32 - 1)) > 0
        self._lidar_ranging_active = (integer_buffer & 1 << (32 - 2)) > 0
        self._lidar_nfound_ok = (integer_buffer & 1 << (32 - 3)) > 0
        self._lidar_nfound_near_ok = (integer_buffer & 1 << (32 - 4)) > 0
        self._lidar_points_ok = (integer_buffer & 1 << (32 - 5)) > 0
        self._lidar_euler_ok = (integer_buffer & 1 << (32 - 6)) > 0
        self._lidar_fval_ok = (integer_buffer & 1 << (32 - 7)) > 0
        self._lidar_tracking_valid = (integer_buffer & 1 << (32 - 8)) > 0
        self._lidar_solution_ok_sending = (integer_buffer & 1 << (32 - 9)) > 0
        self._lidar_use_pcap = (integer_buffer & 1 << (32 - 10)) > 0
        self._drone.generate_send_lidar_status()

        if self.lidar_should_activate():
            self.activate_lidar()

        # print("VELOSTAT1 ->")
        # print(integer_buffer)
        # print("fval_raw: \t\t\t\t\t\t", fval_raw)
        # print("fval: \t\t\t\t\t\t\t", self._lidar_last_fval)
        # print("max_line_z: \t\t\t\t\t", self._lidar_max_line_z)
        # print("_lidar_streaming : \t\t\t\t", self._lidar_streaming)
        # print("_lidar_ranging_active : \t\t", self._lidar_ranging_active)
        # print("_lidar_nfound_ok : \t\t\t\t", self._lidar_nfound_ok)
        # print("_lidar_nfound_near_ok : \t\t", self._lidar_nfound_near_ok)
        # print("_lidar_points_ok : \t\t\t\t", self._lidar_points_ok)
        # print("_lidar_euler_ok : \t\t\t\t", self._lidar_euler_ok)
        # print("_lidar_fval_ok : \t\t\t\t", self._lidar_fval_ok)
        # print("_lidar_tracking_valid : \t\t", self._lidar_tracking_valid)
        # print("_lidar_solution_ok_sending : \t", self._lidar_solution_ok_sending)
        # print("_lidar_use_pcap : \t\t\t\t", self._lidar_use_pcap)

    def process_mission(self, msg: MAVLink_message) -> MAVLink_message:
        if (msg.get_type() == "MISSION_COUNT") or (msg.get_type() == "MISSION_ITEM_INT") \
                or (msg.get_type() == "MISSION_ITEM"):
            print(msg)
            if msg.get_type() == "MISSION_COUNT":
                msg = cast(MAVLink_mission_count_message, msg)
                if msg.count > 0:
                    msg = cast(MAVLink_mission_count_message, msg)
                    self._mission_count = msg.count
                    self._pylons_buffer = []
                    print(self._mission_count)
            if msg.get_type() == "MISSION_ITEM_INT":
                msg = cast(MAVLink_mission_item_int_message, msg)
                if msg.seq == 0:
                    self._home_buffer = CCoordinate(msg.x, msg.y, msg.z + 9.0)#prev value "+18"
                    print(self._home_buffer.get_position_tuple())
                if msg.command == MAV_CMD_NAV_TAKEOFF:
                    self._takeoff_alt = msg.z
                if msg.seq == 2:
                    self._pylon_alt = msg.z
                if msg.seq > 1:
                    msg.z = self._pylon_alt + self._drone.global_pos.alt
                    msg.frame = 0
                    print(msg)
                    self._pylons_buffer.append(CPylon(CCoordinate(msg.x, msg.y, msg.z)))
                    self._mission_count -= 1
                    print(self._mission_count)
                    if self._mission_count == 2:
                        self._current_mission = CSurveyMission(
                            home_location=self._home_buffer,
                            pylon_list=self._pylons_buffer)
                        self._mission_loaded = True
                        self._current_mission.serialize(self.MISSION_BUFFER_FILENAME)
                        print(self._current_mission.spans)
            if msg.get_type() == "MISSION_ITEM":
                msg = cast(MAVLink_mission_item_int_message, msg)
                if msg.seq == 0:
                    self._home_buffer = CCoordinate(ceil(msg.x * 1e7), ceil(msg.y * 1e7), msg.z + 9.0)#prev value "+18"
                    print(self._home_buffer.get_position_tuple())
                if msg.command == MAV_CMD_NAV_TAKEOFF:
                    self._takeoff_alt = msg.z
                if msg.seq == 2:
                    self._pylon_alt = msg.z
                if msg.seq > 1:
                    msg.z = self._pylon_alt + self._drone.global_pos.alt
                    msg.frame = 0
                    print(msg)
                    self._pylons_buffer.append(CPylon(CCoordinate(ceil(msg.x * 1e7), ceil(msg.y * 1e7), msg.z)))
                    self._mission_count -= 1
                    print(self._mission_count)
                    if self._mission_count == 2:
                        self._current_mission = CSurveyMission(
                            home_location=self._home_buffer,
                            pylon_list=self._pylons_buffer)
                        self._mission_loaded = True
                        self._current_mission.serialize(self.MISSION_BUFFER_FILENAME)
                        print(self._current_mission.spans)
        return msg

    def try_load_mission(self):
        try:
            f = open(self.MISSION_BUFFER_FILENAME, 'r')
            self._current_mission = deserialize_mission(self.MISSION_BUFFER_FILENAME)
            self._mission_loaded = True
            print("Loaded buffered mission: ", self._current_mission)
            f.close()
        except IOError:
            print("No buffered mission available")

    def move_to_next_tower(self):
        if self.drone._distance_to_next_tower > self.stop_distance_to_tower and self._wheels_move_flag == True:
            self.drone.wheels_forward(self._mav, 982)
            self._wheels_move_flag = False
        if self.drone._distance_to_next_tower <= self.stop_distance_to_tower and self._wheels_stop_flag == False:
            self.drone.wheels_forward(self._mav, 1495)
            self._wheels_stop_flag = True
            self._wheels_move_flag = False

    def goto_span_land(self, offset_alt: float):
        print("OFFSET_ALT: ", offset_alt)

        setpoint_pos_corrected = self.current_mission.get_setpoint_position_corrected()
        if isnan(offset_alt):
            setpoint_pos_corrected._alt = self.drone.global_pos.alt
        else:
            setpoint_pos_corrected = setpoint_pos_corrected.offset_alt(offset_alt)
        print(setpoint_pos_corrected)
        setpoint_yaw_corrected = self.current_mission.get_setpoint_yaw_corrected()
        self.drone.generate_send_global_setpoint(setpoint_pos_corrected)
        print("setpoint_yaw_corrected = ", setpoint_yaw_corrected, " / ", degrees(setpoint_yaw_corrected))
        self.drone.generate_send_heading(degrees(setpoint_yaw_corrected))

        lat, lon, alt = self.current_mission.get_correction_pos().get_position_tuple()
        msg = self.drone.connection.generate_pack_debug_vect(bytearray("CORRECTION", "ASCII"), lat, lon, alt)
        self.broadcast_mavlink_message(msg)

    def process_unity(self, msg: MAVLink_named_value_int_message) -> None:
        # print("3 ->", msg)
        if msg.name == "SETNEWSPAN":
            self.current_mission.set_span(msg.value)
            # send packet to Unity from drone system, otherwise Unity drops the packet
            out_msg = self.drone.connection.generate_pack_named_value_int(
                bytearray("SETNEWSPAN", "ASCII"), self.current_mission.get_span())
            self.broadcast_mavlink_message(out_msg)

        if msg.name == "STARTDIAGN":    #добавил 2021/09/21
            self.current_mission.get_current_span().start_autodiagnonstic()#добавил 2021/09/21

        if msg.name == "STOPDIAGNO":    #добавил 2021/09/21
            self.current_mission.get_current_span().stop_autodiagnonstic()#добавил 2021/09/21

        if msg.name == "UPYLONOFST":    #добавил
            self.current_mission.get_current_span().set_pylon_offset(msg.value)#добавил

        if msg.name == "DIST_TOWER":
            self.stop_distance_to_tower = msg.value

        if msg.name == "WHEELSMOVE":  # добавил
            self._wheels_move_flag = True
            self._wheels_stop_flag = False
            #self.current_mission.get_current_span().move_wheels_to_next_waypoint(msg.value)  # добавил
            #self.drone.pantorgraf_servo(self._mav)# добавил 2022-11-21

        if msg.name == "ALTOVRWIRE":  # добавил
            self._takeoff_alt_from_wire = msg.value  # добавил

        if msg.name == "UNITYUNITY":
            # Arm
            if msg.value == 101:
                self.drone.arm(self._mav)

            # Disarm
            elif msg.value == 100:
                self.drone.disarm(self._mav)

            # Takeoff to 15 meters
            elif msg.value == 111:
                self.is_taking_off = True
                self.auto_land = False
                self.full_land = False
                self.drone.takeoff(self._takeoff_alt, self._mav)

            # Takeoff from wire to 9 meters (previous value: 7 meters)
            elif msg.value == 163:
                self.is_taking_off = True
                self.auto_land = False
                self.full_land = False
                self.drone.takeoff(self._takeoff_alt_from_wire, self._mav)

            # Land flight mode
            elif msg.value == 110:
                # TODO update other flags if needed
                self.drone.land(self._mav)

            # Guided flight mode
            elif msg.value == 120:
                self.drone.set_flight_mode("GUIDED")

            # Go to home using preset altitude
            elif msg.value == 130:
                self.drone.generate_send_global_setpoint(self.current_mission.get_home_coordinate_3d())
                self.drone.generate_send_heading(degrees(self.current_mission.get_current_heading()))

            # Advance mission span
            elif msg.value == 131:
                self.current_mission.advance_span()
                # send packet to Unity from drone system, otherwise Unity drops the packet
                out_msg = self.drone.connection.generate_pack_named_value_int(
                    self, bytearray("SETNEWSPAN", "ASCII"), self.current_mission.get_span())
                self.broadcast_mavlink_message(out_msg)

            # Retreat mission span
            elif msg.value == 134:
                self.current_mission.retreat_span()
                # send packet to Unity from drone system, otherwise Unity drops the packet
                out_msg = self.drone.connection.generate_pack_named_value_int(
                    self, bytearray("SETNEWSPAN", "ASCII"), self.current_mission.get_span())
                self.broadcast_mavlink_message(out_msg)

            # Approach current span landing with LVL1/base altitude
            elif msg.value == 135:
                # GoToLandA
                self._auto_navigation_mode = 0
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = 6.0
                self.goto_span_land(nan)

            # Approach current span landing with LVL1/base altitude
            elif msg.value == 132:
                # GoToLandB
                self._auto_navigation_mode = 0
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = 6.0
                self.goto_span_land(self.offset_alt)

            # Approach current span landing with LVL1/base altitude automatic
            elif msg.value == 137:
                # GoToLandBC
                self._auto_navigation_mode = 1
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = 6.0
                self.goto_span_land(self.offset_alt)

            # Approach current span landing with LVL2 altitude GoToLandC()
            elif msg.value == 133:
                # GoToLandC
                self._auto_navigation_mode = 0
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = 4.0
                self.goto_span_land(self.offset_alt)

            # Approach current span landing with HIGH altitude
            elif msg.value == 162:
                # GoToLand High
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = 10.0
                # self.goto_span_land(self.offset_alt)
                self.goto_span_land(nan)

            # Approach current span landing and increase altitude
            elif msg.value == 151:
                # GoToLand Up
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = self.offset_alt + 0.25
                self.goto_span_land(self.offset_alt)

            # Approach current span landing and decrease altitude
            elif msg.value == 152:
                # GoToLand Down
                self.is_taking_off = False
                self.auto_land = False
                self.full_land = False
                self.offset_alt = self.offset_alt - 0.25
                self.goto_span_land(self.offset_alt)

            # Engage autoland on wire
            elif msg.value == 161:
                # Auto Land
                if self.auto_land_available:
                    self.auto_land = True
                    self.full_land = False

            elif msg.value == 171:
                land_position = self.current_mission.get_current_land_coordinate_3d()
                drone_position = self.drone.global_pos
                distance = land_position.get_2d_distance_to(drone_position)
                heading_span = self.current_mission.get_current_heading()
                heading_drone = land_position.get_bearing_to(drone_position)
                heading_drone -= heading_span
                if heading_drone > pi:
                    heading_drone -= 2 * pi
                if heading_drone < -pi:
                    heading_drone += 2 * pi

                x = 0.0
                y = distance if heading_drone >= 0 else -distance
                z = drone_position.alt - land_position.alt
                msg = self.drone.connection.generate_pack_debug_vect(bytearray("LDR_ACTIV", "ASCII"), 0.0, y, z)
                self.broadcast_mavlink_message(msg)

            elif msg.value == 172:
                msg = self.drone.connection.generate_pack_debug_vect(bytearray("LDR_DEACT", "ASCII"), 0.0, 0.0, 0.0)
                self.broadcast_mavlink_message(msg)
