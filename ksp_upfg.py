import krpc
import time
import math
import argparse
import numpy as np

import settings
from utilities import * 
from AtmosphericFlight import wait_for_launch, atmospheric_flight_control
from flight_time_window import FlightTimeWindow
import upfg
from vessel_state import VesselState

def delay(state:VesselState, t:float):
   t -= 0.02
   ts = state.universal_time()
   elapse = 0.0
   while elapse < t:
      time.sleep(0.001)
      elapse = state.universal_time() - ts

def draw_thrust_vector(vessel:Vessel, vector:np.ndarray):
   clear_lines(vessel)
   vector = normalize_vector(vector) 
   draw_vector(vessel, vector, (1, 0, 0), vessel.orbit.body.non_rotating_reference_frame, 100)

def angle_from_vec(vessel:Vessel, x:list, angle:str):
   east = [0, 0, 1]
   north = [0, 1, 0]
   up = [1, 0, 0]

   space_center = vessel._client.space_center
   ref = vessel.orbit.body.non_rotating_reference_frame

   surface_frame = vessel.surface_reference_frame
   vector = space_center.transform_direction(x, ref, surface_frame)

   if angle == 'pitch':
      return 90 - math.degrees(angle_between_vectors(up, vector))
   elif angle == 'yaw':
      out = math.degrees(math.atan2(np.vdot(east, vector), np.vdot(north, vector)))
      if out < 0.0:
         out += 360.0
      return out
   
def closed_loop_guidance(vessel:Vessel, upfg_target:dict):
   print('Starting UPFG')

   vehicle = upfg.analyze_vehicle(vessel)
   for stage in vehicle:
      print(f'stage: m0={stage.m0}, fT={stage.fT}, ve={stage.ve}, l1={stage.l1}')

   if len(vehicle) != 2:
      raise Exception('UPFG: Invalid number of stages')  
   
   vehicle.pop()
   
   upfg_internal = upfg.upfg_initial(vessel_state, upfg_target)
   upfg_guided = upfg.Struct()
   converged = False
   iteration = int(0)

   while converged is False:
      delay(vessel_state, 0.1)

      [upfg_internal, upfg_guided] = upfg.upfg_control(vehicle, upfg_target, upfg_internal, vessel_state)
      t1 = upfg_guided.tgo

      draw_thrust_vector(vessel, upfg_guided.i_f)

      delay(vessel_state, 0.1)

      [upfg_internal, upfg_guided] = upfg.upfg_control(vehicle, upfg_target, upfg_internal, vessel_state)
      t2 = upfg_guided.tgo

      draw_thrust_vector(vessel, upfg_guided.i_f)
      
      if abs(t1 - t2) / t2 < 0.01:
         converged = True

      iteration += 1

   print(f'Guidance converged after {iteration} iteration')

   while True:
      [upfg_internal, upfg_guided] = upfg.upfg_control(vehicle, upfg_target, upfg_internal, vessel_state)

      if upfg_guided.tgo > 1.0:
         pitch = angle_from_vec(vessel, upfg_guided.i_f, 'pitch')
         yaw = angle_from_vec(vessel, upfg_guided.i_f, 'yaw')

         vessel.auto_pilot.target_heading = yaw
         vessel.auto_pilot.target_pitch = pitch
         vessel.auto_pilot.target_roll = 0.0

         draw_thrust_vector(vessel, upfg_guided.i_f)

      if upfg_guided.tgo < 0.1:
         vessel.control.throttle = 0.0
         break

      if upfg_guided.tgo < 10.0:
         vessel.control.throttle = 0.1

      #if vessel_state.speed_eci() > upfg_target.velocity:
      #   vessel.control.throttle = 0.0
      #   break

      print(f'utc: {vessel_state.universal_time()}, tgo: {upfg_guided.tgo:.2f}, orbit speed: {vessel_state.speed_eci():.2f}, target orbit speed: {upfg_target.velocity:.2f}')
      delay(vessel_state, 0.1)

parser = argparse.ArgumentParser()
parser.add_argument('--upfg_debug', action='store_true')
args = parser.parse_args()

client = krpc.connect(name='Launch to orbit')
vessel = client.space_center.active_vessel
print(vessel.name)

vessel_state = VesselState(vessel)

upfg_target = upfg.setup_upfg_target()

if args.upfg_debug:
   vessel.auto_pilot.target_pitch_and_heading(5.0, 90.0)
   vessel.auto_pilot.engage()
   vessel.control.throttle = 1.0
   vessel.control.activate_next_stage()
   time.sleep(10)

   closed_loop_guidance(vessel, upfg_target)

   print('Mission Success')
else:
   long, lat = geo_position(vessel)
   print(f'Geo position: {long}, {lat}')
   settings.mission['inclination'] = lat

   intercept_time = orbit_intercept_time(vessel, settings.mission['direction'], settings.mission['inclination'], settings.mission['LAN'])
   liftoff_time = int(client.space_center.ut + intercept_time - settings.mission['launch_time_advance'])
   print(f'Intercept time: {intercept_time}, Liftoff time: {liftoff_time}')

   ft = FlightTimeWindow(liftoff_time)

   wait_for_launch(vessel, liftoff_time, True)

   atmospheric_flight_control(vessel, upfg_target)

   closed_loop_guidance(vessel, upfg_target)

   print('Mission Success')

while True:
   time.sleep(1)