import krpc
from krpc.services.spacecenter import *
import time
import math
import numpy as np

import settings
from utilities import *
from vessel_state import VesselState

class FlightState:
   takeoff = 0,
   roll_maneuver = 1,
   pitch_maneuver = 2,
   gravity_turn = 4,
   wait_stage1_separated = 5,

def wait_for_launch(vessel:Vessel, liftoff_time:int, indicator=False):
   client = vessel._client

   refresh_counter = 0
   current_time = int(client.space_center.ut)

   client.space_center.warp_to(liftoff_time - 10)

   while current_time < liftoff_time:
      if indicator:
         refresh_counter += 1
         if refresh_counter == 10:
            refresh_counter = 0

            client.drawing.clear()

            position = cartesian_position(vessel, vessel.orbit.body.reference_frame) # ECEF position
            ls_normal = normalize_vector(np.array(position))
            earth = vessel.orbit.body
            line = client.drawing.add_direction_from_com(ls_normal, earth.reference_frame, 200)
            line.color = (1, 0, 0)
            line.thickness = 1
  
            orbit_normal = target_normal_ECEF(vessel, settings.mission['inclination'], settings.mission['LAN'])

            line = client.drawing.add_direction_from_com(orbit_normal.tolist(), earth.reference_frame, 200)
            line.color = (0, 1, 0)
            line.thickness = 1

      time.sleep(0.01)
      current_time = int(client.space_center.ut)

   if indicator:
      client.drawing.clear()   
      
def atmospheric_flight_control(vessel:Vessel, upfg_target, state:VesselState):
   target_azimuth = launch_azimuth(vessel, upfg_target.velocity, settings.mission['inclination'], upfg_target.angle)
   print(f'Target azimuth: {target_azimuth:.2f}')
   initial_azi = initial_azimuth(vessel)
   print(f'Initial azimuth: {initial_azi:.2f}')

   vessel.control.throttle = 1.0
   vessel.control.sas = False
   vessel.control.rcs = False

   vessel.auto_pilot.target_pitch_and_heading(90.0, initial_azi)
   vessel.auto_pilot.engage()

   vessel.control.activate_next_stage() # main engine ignition
   stage1_ignition_time = state.universal_time()

   while state.thrust() < state.available_thrust() * 0.9:
      pass

   vessel.control.activate_next_stage() # booster ignition
   srb_ignition_time = state.universal_time()

   delay(state, 0.1)
   vessel.control.activate_next_stage() # lift off
   liftoff_time =state.universal_time()
   print(f'Liftoff at {liftoff_time:.2f}')

   flight_state = FlightState.takeoff

   booster_separated = False  
   stage1_separated = False
   finished = False

   start_pitch_manuever = False
   pitch_done = False   

   while not finished:
      current_time = state.universal_time()
      
      if not booster_separated and current_time - srb_ignition_time >= settings.booster_burn_time:
         print('Booster separation')
         vessel.control.activate_next_stage()
         booster_separated = True

      if not stage1_separated and current_time - stage1_ignition_time >= settings.first_stage_burn_time:
         print('Stage 1 separation')
         stage1_separated = True
         vessel.control.throttle = 0.0
         delay(state, 1.0)
         vessel.control.activate_next_stage()
         delay(state, 2.0)
         vessel.control.throttle = 1.0
         vessel.control.activate_next_stage()
         
      match flight_state:
         case FlightState.takeoff:
            if state.mean_altitude() > settings.roll_altitude:
               flight_state = FlightState.roll_maneuver
         case FlightState.roll_maneuver:
            azi = initial_azimuth(vessel)
            if (math.fabs(azi - target_azimuth) < 1.0):
               flight_state = FlightState.pitch_maneuver
               pitch_angle = 90.0
            else:   
               vessel.auto_pilot.target_pitch_and_heading(90, target_azimuth)       
         case FlightState.pitch_maneuver: 
            if not start_pitch_manuever:
               if state.vertical_speed() > settings.pitch_velocity:
               #if state.mean_altitude() > 500.0:
                  start_pitch_manuever = True
                  pitch_start_time = current_time

                  pitch_angle -= settings.pitch_rate
                  vessel.auto_pilot.target_pitch_and_heading(pitch_angle, target_azimuth)   
            else:      
               if current_time - pitch_start_time > 1.0:
                  pitch_start_time = current_time

                  if not pitch_done:
                     if math.fabs(pitch_angle - 90.0 + settings.pitch_maneuver_angle) < 0.5:
                        pitch_done = True
                     else:
                        pitch_angle -= 0.5
                        vessel.auto_pilot.target_pitch_and_heading(pitch_angle, target_azimuth)   
                  else:      
                     if math.fabs(state.angle_of_attack()) < 0.5:
                        flight_state = FlightState.gravity_turn
                        vessel.auto_pilot.target_roll = 0.0
         case FlightState.gravity_turn:            
            diff = state.pitch() - state.angle_of_attack()
            vessel.auto_pilot.target_pitch_and_heading(diff, target_azimuth) 
            if math.fabs(state.pitch() - settings.target_picth) < 1.0:
               vessel.auto_pilot.target_pitch_and_heading(settings.target_picth, target_azimuth)
               flight_state = FlightState.wait_stage1_separated

            if stage1_separated:
               vessel.auto_pilot.target_pitch_and_heading(state.pitch(), target_azimuth)
               finished = True
         case FlightState.wait_stage1_separated:   
            if stage1_separated:   
               finished = True

      time.sleep(0.01)  

   print('atmospheric flight control finished')   