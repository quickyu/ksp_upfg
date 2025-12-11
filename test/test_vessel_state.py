import sys 
sys.path.append("..") 
import time
import krpc
from vessel_state import *

client = krpc.connect(name='Test')
vessel = client.space_center.active_vessel
print(vessel.name)

vessel_state = VesselState(vessel)

while True:
   print(f'position: {vessel_state.position()}')
   print(f'velocity: {vessel_state.velocity()}')
   print(f'thrust: {vessel_state.thrust()}')
   print(f'mass: {vessel_state.mass()}')
   print(f'isp: {vessel_state.specific_impulse()}')
   print(f'ut: {vessel_state.universal_time()}\n')
   time.sleep(1)