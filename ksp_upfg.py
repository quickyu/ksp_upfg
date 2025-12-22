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

def draw_thrust_vector(vessel:Vessel, vector:np.ndarray, color:tuple):
   clear_lines(vessel)
   vector = normalize_vector(vector) 
   draw_vector(vessel, vector, color, vessel.orbit.body.non_rotating_reference_frame, 60)

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

def enable_rcs(vessel:Vessel, enable:bool):
   if enable:
      for rcs in vessel.parts.rcs:
         rcs.enabled = True
         rcs.pitch_enabled = True
         rcs.yaw_enabled = True
         rcs.roll_enabled = True
      vessel.control.rcs = True
   else:
      vessel.control.rcs = False  

def closed_loop_guidance(vessel:Vessel, upfg_target:dict, state:VesselState):
   print('Starting UPFG')

   vehicle = upfg.analyze_vehicle(vessel)
   for stage in vehicle:
      print(f'stage: m0={stage.m0}, fT={stage.fT}, ve={stage.ve}, l1={stage.l1}')

   if len(vehicle) != 2:
      raise Exception('UPFG: Invalid number of stages')  
   
   vehicle.pop()
   
   upfg_internal = upfg.upfg_initial(state, upfg_target)
   upfg_guided = upfg.Struct()

   current_time = state.universal_time()
   last_time = state.universal_time()

   upfg_converged = False
   c_count = 0
   iteration = int(0)
   last_tgo = upfg_internal.tgo

   color = (1.0, 0.0, 0.0)

   last_vector = np.zeros(3)

   def reset_upfg():
      print('Reset UPFG')
      nonlocal upfg_internal, upfg_converged, iteration, color, last_tgo, last_time
      upfg_internal = upfg.upfg_initial(state, upfg_target)
      last_tgo = upfg_internal.tgo
      last_time = state.universal_time()
      upfg_converged = False
      iteration = 0
      color = (1.0, 0.0, 0.0)

   while True:
      current_time = state.universal_time()

      upfg_internal, upfg_guided = upfg.upfg_control(vehicle, upfg_target, upfg_internal, state)
      draw_thrust_vector(vessel, upfg_guided.i_f, color)

      if not upfg_converged:
         iteration += 1

      delta_t = current_time - last_time
      last_time = current_time

      expected_tgo = last_tgo - delta_t
      last_tgo = upfg_guided.tgo

      if (math.fabs(expected_tgo - upfg_guided.tgo) < 0.5):
         if not upfg_converged:
            c_count += 1
            if c_count > 2:
               c_count = 0
               upfg_converged = True
               color = (0.0, 1.0, 0.0)
               print(f'UPFG converged after {iteration} iteration')
               iteration = 0
         else:
            va = angle_between_vectors(np.asarray(upfg_guided.i_f), last_vector)
            if math.fabs(va) > math.radians(15.0):
               reset_upfg()
      else:
         c_count = 0
         if upfg_converged:
            reset_upfg()

      if upfg_converged:   
         last_vector = np.asarray(upfg_guided.i_f)

         pitch = angle_from_vec(vessel, upfg_guided.i_f, 'pitch')
         yaw = angle_from_vec(vessel, upfg_guided.i_f, 'yaw')

         vessel.auto_pilot.target_heading = yaw
         vessel.auto_pilot.target_pitch = pitch
         vessel.auto_pilot.target_roll = 0.0

      if upfg_converged and upfg_guided.tgo < 3.0:
         end_time = current_time + upfg_guided.tgo
         while state.universal_time() < end_time:  
            time.sleep(0.001)
         vessel.control.throttle = 0.0
         break

      if state.speed_eci() > upfg_target.velocity:
         vessel.control.throttle = 0.0
         break

      print(f'utc: {state.universal_time()}, tgo: {upfg_guided.tgo:.2f}, orbit speed: {state.speed_eci():.2f}, target orbit speed: {upfg_target.velocity:.2f}')
      delay(state, 0.1)  

   clear_lines(vessel)   

def flight():
   parser = argparse.ArgumentParser()
   parser.add_argument('--upfg_debug', action='store_true')
   args = parser.parse_args()

   client = krpc.connect(name='Launch to orbit')
   vessel = client.space_center.active_vessel
   print(vessel.name)

   vessel_state = VesselState(vessel)

   upfg_target = upfg.setup_upfg_target()

   if args.upfg_debug:
      vessel.control.throttle = 1.0
      vessel.control.activate_next_stage()
      vessel.auto_pilot.target_pitch_and_heading(settings.target_picth, 90.0)
      vessel.auto_pilot.engage()
      time.sleep(5)

      closed_loop_guidance(vessel, upfg_target, vessel_state)
   else:
      long, lat = geo_position(vessel)
      print(f'Geo position: {long}, {lat}')
      settings.mission['inclination'] = lat

      intercept_time = orbit_intercept_time(vessel, settings.mission['direction'], settings.mission['inclination'], settings.mission['LAN'])
      liftoff_time = int(client.space_center.ut + intercept_time - settings.mission['launch_time_advance'])
      print(f'Intercept time: {intercept_time}, Liftoff time: {liftoff_time}')

      ft = FlightTimeWindow(liftoff_time)

      wait_for_launch(vessel, liftoff_time, True)

      atmospheric_flight_control(vessel, upfg_target, vessel_state)

      time.sleep(5)

      closed_loop_guidance(vessel, upfg_target, vessel_state)

      equatorial_radius = vessel.orbit.body.equatorial_radius
      print(f'Orbital elements: apoapsis={(vessel.orbit.apoapsis - equatorial_radius):.6f}, periapsis={(vessel.orbit.periapsis - equatorial_radius):.6f}, '
         f'inclination={math.degrees(vessel.orbit.inclination):.6f}, RAAN={math.degrees(vessel.orbit.longitude_of_ascending_node):.6f}, '
         f'period={vessel.orbit.period:.2f}')

      enable_rcs(vessel, True)

      space_center = client.space_center

      while True:
         direction = space_center.transform_direction((0., 1., 0.), vessel.orbital_reference_frame,  vessel.surface_reference_frame)
         if direction[1] == 0.0:
            heading = 90.0 if direction[2] > 0.0 else -90.0
         else:   
            heading = math.degrees(math.atan2(direction[2], direction[1]))
         print(f'Heading: {heading}')   

         vessel.auto_pilot.target_roll = 0.0
         vessel.auto_pilot.target_pitch = 0.0
         vessel.auto_pilot.target_heading = heading  

   print('Mission Success')

   while True:
      time.sleep(1)

if __name__ == '__main__':
   flight()   