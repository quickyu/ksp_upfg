import krpc
import time
import numpy as np

import settings
from utilities import * 
from atmo_control import *
from flight_time_window import *
from upfg import *

client = krpc.connect(name='Launch to orbit')
vessel = client.space_center.active_vessel
print(vessel.name)

setup_upfg_target(vessel)

intercept_time = orbit_intercept_time(vessel, settings.mission['direction'], np.radians(settings.mission['inclination']), np.radians(settings.mission['LAN']))
liftoff_time = int(client.space_center.ut + intercept_time - settings.mission['launch_time_advance'])
print(f'Intercept time: {intercept_time}, Liftoff time: {liftoff_time}')

ft = FlightTimeWindow(liftoff_time)

wait_for_launch(vessel, liftoff_time, True)

atmospheric_control(vessel, upfg_target)

while True:
   time.sleep(1)