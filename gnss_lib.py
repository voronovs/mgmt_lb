# Some code adapted from https://github.com/dronekit/dronekit-python/blob/master/examples/guided_set_speed_yaw/guided_set_speed_yaw.py

from math import *


class CCoordinate:
    _lat: int
    _lon: int
    _alt: float
    # DY DO NOT reload arithmetic operator - : would lead to confusion for 2d/3d distances

    def __init__(self, lat, lon, alt):
        self._lat = lat
        self._lon = lon
        self._alt = alt

    def __add__(self, o):
        lat2, lon2, alt2 = o.get_position_tuple()
        return CCoordinate(self._lat + lat2, self._lon + lon2, self._alt + alt2)

    def __sub__(self, o):
        lat2, lon2, alt2 = o.get_position_tuple()
        return CCoordinate(self._lat - lat2, self._lon - lon2, self._alt - alt2)

    def __str__(self):
        return "CCoordinate({:.7f} : {:.7f} : {:.2f})".format(self._lat * 1e-7, self._lon * 1e-7, self._alt)

    def get_3d_distance_to(self, location2):
        distance_2d = self.get_2d_distance_to(location2)
        lat2, lon2, alt2 = location2.get_position_tuple()
        delta_alt = self._alt - alt2
        return sqrt(distance_2d * distance_2d + delta_alt * delta_alt)

    def get_position_tuple(self):
        return self._lat, self._lon, self._alt

    def get_position_tuple_float(self):
        return self._lat * 1e-7, self._lon * 1e-7, self._alt

    """
    Functions to make it easy to convert between the different frames-of-reference. In particular these
    make it easy to navigate in terms of "metres from the current position" when using commands that take
    absolute positions in decimal degrees.
    The methods are approximations only, and may be less accurate over longer distances, and when close
    to the Earth's poles.
    Specifically, it provides:
    * get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
    * get_distance_metres - Get the distance between two LocationGlobal objects in metres
    * get_bearing - Get the bearing in degrees to a LocationGlobal
    """

    def get_bearing_to(self, location2):

        lat1, lon1, alt1 = self.get_position_tuple_float()
        lat2, lon2, alt2 = location2.get_position_tuple_float()

        lat_now_rad = radians(lat1)
        lat_next_rad = radians(lat2)

        cos_lat_next = cos(lat_next_rad)
        d_lon = radians(lon2 - lon1)

        y = sin(d_lon) * cos_lat_next
        x = cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next * cos(d_lon)

        bearing = atan2(y, x)
        if bearing > 2*pi:
            bearing -= 2*pi
        if bearing < 0.0:
            bearing += 2*pi
        
        return bearing


    def get_2d_distance_to(self, location2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """

        lat1, lon1, alt1 = self.get_position_tuple_float()
        lat2, lon2, alt2 = location2.get_position_tuple_float()

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1

        return sqrt((delta_lat * delta_lat) + (delta_lon * delta_lon)) * 1.113195e5

    def offset_by(self, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth

        lat, lon, alt = self.get_position_tuple_float()
        # print(lat, lon, alt)

        # Coordinate offsets in radians
        delta_lat = dNorth / earth_radius
        delta_lon = dEast / (earth_radius * cos(pi * lat / 180))

        # New position in decimal degrees
        new_lat = lat + (delta_lat * 180 / pi)
        new_lon = lon + (delta_lon * 180 / pi)

        return CCoordinate(ceil(new_lat * 1e7), ceil(new_lon * 1e7), alt)

    def offset_along_heading(self, heading, distance):
        delta_north = cos(heading) * distance
        delta_east = sin(heading) * distance
        # print(delta_north, delta_east)
        return self.offset_by(delta_north, delta_east)

    def offset_alt(self, delta_alt: float):
        return CCoordinate(self._lat, self._lon, self._alt + delta_alt)

    @property
    def alt(self):
        return self._alt
