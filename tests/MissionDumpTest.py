from nav_static import *
from gnss_lib import *

home_buffer = CCoordinate(1, 2, 3.0)

pylons_buffer = [CPylon(CCoordinate(11, 22, 33.0)), CPylon(CCoordinate(111, 122, 333.0)),
                 CPylon(CCoordinate(1111, 2222, 33333.0))]

current_mission = CSurveyMission(
    home_location=home_buffer,
    pylon_list=pylons_buffer)

print(current_mission)

current_mission.serialize('test_mission.json')

current_mission2 = deserialize_mission('test_mission.json')
print(current_mission2)
