import krpc
import time
import numpy as np

import settings
from utilities import * 
from flight_manager import *
from flight_time_window import *
import upfg

client = krpc.connect(name='Launch to orbit')
vessel = client.space_center.active_vessel
print(vessel.name)

upfg_target = upfg.setup_upfg_target(vessel)
print(f'UPFG target: {upfg_target}')

long, lat = geo_position(vessel)
print(f'Geo position: {long}, {lat}')
settings.mission['inclination'] = lat

intercept_time = orbit_intercept_time(vessel, settings.mission['direction'], np.radians(settings.mission['inclination']), np.radians(settings.mission['LAN']))
liftoff_time = int(client.space_center.ut + intercept_time - settings.mission['launch_time_advance'])
print(f'Intercept time: {intercept_time}, Liftoff time: {liftoff_time}')

ft = FlightTimeWindow(liftoff_time)

wait_for_launch(vessel, liftoff_time, True)

open_loop_guidance(vessel, upfg_target)

while True:
   time.sleep(1)