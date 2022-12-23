from typing import List
from gnss_lib import *
import jsonpickle

from move import WireMover
from diagnosticproxy import DiagnosticProxy


class CPylon:
    def __init__(self, coordinate: CCoordinate):
        self._coordinate = coordinate

    def get_coordinate(self) -> CCoordinate:
        return self._coordinate

    def __str__(self):
        lat, lon, alt = self._coordinate.get_position_tuple_float()
        return "CPylon({:.7f} : {:.7f} : {:.2f})".format(lat, lon, alt)


class CSpan:
    _pylon_start: CPylon
    _pylon_end: CPylon

    def __init__(self, pylon_start: CPylon, pylon_end: CPylon):
        self._pylon_start = pylon_start
        self._pylon_end = pylon_end
        self._heading = self.calculate_bearing()
        self._pylon_offset = 15 #добавил
        print("get_bearing_to: ", self._pylon_start.get_coordinate().get_bearing_to(self._pylon_end.get_coordinate()))

    def __str__(self):
        return "CSpan({:} : {:})".format(self._pylon_start, self._pylon_end)

    def set_pylon_offset(self, new_pylon_offset):   #добавил
        self._pylon_offset = new_pylon_offset   #добавил
        print("Pylon offset sent ", self._pylon_offset) #добавил

    def start_autodiagnonstic(self):   #добавил 2021/09/21
        dproxy = DiagnosticProxy(self.callback_fun)
# Функция не блокирующая! Для отслеживания завершения использовать коллбэк
        res = dproxy.do_diagnostic()
        print('Command sent: {}'.format(res))
        print("Autodiagnostic started") #добавил 2021/09/21

    def stop_autodiagnonstic(self):   #добавил 2021/09/21
        dproxy = DiagnosticProxy(self.callback_fun)
        res = dproxy.halt()
        print('Command sent: {}'.format(res))
        print("Autodiagnostic stopped") #добавил 2021/09/21

    def callback_fun(self): #вызовется, когда платформа доедет

        print('Done callback!') #добавил

    def move_wheels_to_next_waypoint(self, empty):   #функция движения по проводу
        # Для каждого вызова необходимо создавать новый экземпляр WireMover
        wm = WireMover(self.callback_fun)

        # Функция не блокирующая! Для отслеживания завершения использовать коллбэк
        # В случае успеха подачи команды вернет True
        res = wm.move_to_next_tower()
        print('Command sent: {}'.format(res))

        self._moving_by_wheels = empty  # добавил
        print("Move via wheels command sent ", self._moving_by_wheels)  # добавил

    def get_land_coordinate_3d(self) -> CCoordinate:
        # print(self._pylon_start.get_coordinate().get_position_tuple())
        # print(self._pylon_end.get_coordinate().get_position_tuple())
        return self._pylon_start.get_coordinate().offset_along_heading(self._heading, self._pylon_offset) #изменил

    def get_takeoff_coordinate_3d(self) -> CCoordinate:
        return self._pylon_end.get_coordinate().offset_along_heading(self._heading, -5.0)

    def calculate_bearing(self):
        # TODO Test
        return self._pylon_start.get_coordinate().get_bearing_to(self._pylon_end.get_coordinate())

    def get_heading(self) -> float:
        return self._heading

    # TODO use current correction for proximity calculation
    def is_land_proximity(self, drone_coordinate: CCoordinate, proximity_radius: float = 3.0):
        return self.get_land_coordinate_3d().get_2d_distance_to(drone_coordinate) < proximity_radius

    # TODO change correction to float vector
    # TODO calculate correction from Lidar position
    def get_correction(self, drone_coordinate: CCoordinate) -> CCoordinate:
        if self.is_land_proximity(drone_coordinate):
            target_lat, target_lon, target_alt = self.get_land_coordinate_3d().get_position_tuple()
            drone_lat, drone_lon, drone_alt = drone_coordinate.get_position_tuple()
            print("Correction: ", drone_lat - target_lat, drone_lon - target_lon, drone_alt - target_alt)
            return CCoordinate(drone_lat - target_lat, drone_lon - target_lon, drone_alt - target_alt)
        else:
            return CCoordinate(0, 0, 0)


def deserialize_mission(filename) -> 'CSurveyMission':
    with open(filename, 'r') as file:
        json_encoded = file.read()
        return jsonpickle.decode(json_encoded)


class CSurveyMission:
    _correction_pos: CCoordinate
    _correction_yaw: float
    _drone_position: CCoordinate
    _drone_yaw: float
    _lidar_pos_y: float
    _lidar_pos_z: float
    _lidar_pos_yaw: float

    def __init__(self, home_location: CCoordinate, pylon_list: List[CPylon]) -> None:
        self._current_span = 0
        self._home_location = home_location
        self._spans = []
        self._correction_pos = CCoordinate(0, 0, 0)
        self._correction_yaw = 0.0
        self._drone_position = CCoordinate(0, 0, 0)
        self._drone_yaw = 0.0
        self._lidar_pos_y = 0.0
        self._lidar_pos_z = 0.0
        self._lidar_pos_yaw = 0.0
        if pylon_list is not None:
            for i in range(1, len(pylon_list)):
                self._spans.append(CSpan(pylon_list[i - 1], pylon_list[i]))
                
    def __str__(self):
        out = "CSurveyMission(Spans={}, ".format(len(self._spans))
        for i in range(0, len(self._spans)):
            out += "{:}, ".format(self._spans[i])
        return out

    def advance_span(self):
        print(self._current_span, len(self._spans))
        if self._current_span < (len(self._spans) - 1):
            self._current_span = self._current_span + 1

    def retreat_span(self):
        if self._current_span > 0:
            self._current_span = self._current_span - 1

    def set_span(self, new_span):
        if len(self._spans) > new_span >= 0:
            self._current_span = new_span
        print("Span sent ", self._current_span)

    def get_span(self) -> int:
        return self._current_span

    def get_current_span(self) -> CSpan:
        return self._spans[self._current_span]

    # TODO use correction in land point calculation
    def get_current_land_coordinate_3d(self) -> CCoordinate:
        return self.get_current_span().get_land_coordinate_3d()

    def get_current_heading(self) -> float:
        return self.get_current_span().get_heading()

    def get_home_coordinate_3d(self) -> CCoordinate:
        return self._home_location

    def set_drone_position_global_int(self, location: CCoordinate):
        self._drone_position = location

    def set_drone_yaw(self, yaw: float):
        self._drone_yaw = yaw

    def set_lidar_position(self, lidar_y: float, lidar_z: float, lidar_yaw: float):
        self._lidar_pos_y = lidar_y
        self._lidar_pos_z = lidar_z
        self._lidar_pos_yaw = lidar_yaw  # 1.5 * lidar_yaw

    def is_land_proximity(self):
        return self.get_current_span().is_land_proximity(self.get_drone_position_corrected())

    def calculate_correction_pos_yaw(self):
        if self.is_land_proximity():
            # Calculate true yaw setpoint in global frame:
            #   Should be lidar_yaw = global_yaw + correction_yaw - target_yaw
            #   <=> correction_yaw = target_yaw - global_yaw + lidar_yaw
            self._correction_yaw = self.get_current_heading() - self._drone_yaw + self._lidar_pos_yaw

            # Piece 1: distance from cable to drone in YZ (lidar_pos)
            # Piece 2: distance from drone to setpoint (local_pos - target_pos)
            # Step 2. calculate true position setpoint in global frame:
            #   lidar_pos = global_pos + correction_pos - target_pos
            #   <=> correction_pos = target_pos - global_pos + lidar_pos
            target_pos: CCoordinate = self.get_current_land_coordinate_3d()
            # print("0. ", self._lidar_pos_y, self._lidar_pos_z)
            # print("1. ", target_pos.get_position_tuple())
            # print("2.1. ", self.get_setpoint_yaw_corrected())
            # print("2.2. ", self.get_setpoint_yaw_corrected() - pi / 2)
            offset_pos: CCoordinate = \
                target_pos.offset_along_heading(self.get_setpoint_yaw_corrected() - pi / 2, -self._lidar_pos_y)
            # print("3. ", offset_pos.get_position_tuple())
            offset_pos = offset_pos.offset_alt(self._lidar_pos_z)
            # print("4. ", offset_pos.get_position_tuple())
            # print("5. ", self._drone_position.get_position_tuple())
            offset_pos = offset_pos - self._drone_position
            print("6. ", offset_pos.get_position_tuple())
            self._correction_pos = offset_pos

    def get_correction_yaw(self):
        return self._correction_yaw

    def get_correction_pos(self) -> CCoordinate:
        return self._correction_pos

    def get_drone_yaw_corrected(self) -> float:
        return self._drone_yaw + self.get_correction_yaw()

    def get_drone_position_corrected(self) -> CCoordinate:
        # print(self.get_correction_pos().get_position_tuple())
        # print(self._drone_position.get_position_tuple())
        return self._drone_position + self.get_correction_pos()

    def get_setpoint_yaw_corrected(self) -> float:
        print("YAW: ", \
              self.get_current_heading(), \
              self._drone_yaw, \
              self._lidar_pos_yaw, \
              self._correction_yaw, \
              self.get_current_heading() - self.get_correction_yaw())
        return self.get_current_heading() - self.get_correction_yaw()

    def get_setpoint_position_corrected(self):
        return self.get_current_land_coordinate_3d() - self.get_correction_pos()

    @property
    def spans(self):
        return self._spans

    def serialize(self, filename):
        json_encoded = jsonpickle.encode(self)
        with open(filename, 'w') as file:
            file.write(json_encoded)
