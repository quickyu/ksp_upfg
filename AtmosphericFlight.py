import krpc
from krpc.services.spacecenter import *
import time
import math
import numpy as np

from utilities import *
from vessel_state import VesselState

class FlightState:
   takeoff = 0,
   roll_maneuver = 1,
   pitch_maneuver = 2,
   gravity_turn = 4,
   wait_stage1_separated = 5,

def wait_for_launch(vessel:Vessel, state:VesselState, ignition_time:int, **kwargs):
   indicator = False
   target_inc = 0.0
   target_lan = 0.0
   if 'indicator' in kwargs and 'target_inc' in kwargs and 'target_lan' in kwargs:
      indicator = kwargs['indicator']
      target_inc = kwargs['target_inc']
      target_lan = kwargs['target_lan']

   client = vessel._client

   refresh_counter = 0
   current_time = state.universal_time()

   client.space_center.warp_to(ignition_time - 5)
      
   while current_time < ignition_time:
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
  
            orbit_normal = target_normal_ECEF(vessel, target_inc, target_lan)

            line = client.drawing.add_direction_from_com(orbit_normal.tolist(), earth.reference_frame, 200)
            line.color = (0, 1, 0)
            line.thickness = 1

      time.sleep(0.01)
      current_time = state.universal_time()

   if indicator:
      client.drawing.clear()   
      
def atmospheric_flight_control(vessel:Vessel, upfg_target, state:VesselState, guidance:list[list], mission_config:dict):
   target_azimuth = launch_azimuth(vessel, upfg_target.velocity, mission_config['inclination'], upfg_target.angle, mission_config['direction'])
   print(f'Target azimuth: {target_azimuth:.2f}')
   initial_azi = initial_azimuth(vessel)
   print(f'Initial azimuth: {initial_azi:.2f}')

   vessel.control.throttle = 1.0
   vessel.control.sas = False
   vessel.control.rcs = False

   vessel.auto_pilot.engage()
   vessel.auto_pilot.target_pitch_and_heading(90.0, initial_azi)

   vessel.control.activate_next_stage() # main engine ignition
   stage1_ignition_time = state.universal_time()

   booster_separated = False  
   liftoff = False
   srb_ignition_time = 0.0
   start_index = int(0)
   
   while True:
      current_time = state.universal_time()
      mission_seconds = current_time - stage1_ignition_time
      print(f'Mission seconds: {mission_seconds:.2f}')

      if not liftoff:
         if  mission_seconds >=guidance[0][-1]:
            raise 'Atmospheric guidance abnormal, mission aborted.'
         
         _, start_index = interp(guidance, mission_seconds, start_index)
         print(start_index, guidance[13][start_index])
         if guidance[13][start_index] == 'Propagate to Tower Clear':
            vessel.control.activate_next_stage() # booster ignition
            srb_ignition_time = state.universal_time()
            delay(state, 0.1)
            vessel.control.activate_next_stage() # lift off
            liftoff_time =state.universal_time()
            print(f'Liftoff at {liftoff_time:.2f}')
            liftoff = True
      else:
         if not booster_separated and current_time - srb_ignition_time >= mission_config['booster_burn_time']:
            print('Booster separation')
            vessel.control.activate_next_stage()
            booster_separated = True

         if mission_seconds <= guidance[0][-1]:
            aoa = state.angle_of_attack()
            pitch_ctrl, start_index = interp(guidance, mission_seconds, start_index)
            print(f'Pitch: {pitch_ctrl:.2f}, AOA: {aoa:.2f}')
            vessel.auto_pilot.target_pitch = pitch_ctrl
            vessel.auto_pilot.target_roll = 0.0
            vessel.auto_pilot.target_heading = target_azimuth
         else:
            break

      delay(state, 0.1)    
       
   print('Stage 1 separation')
   vessel.control.throttle = 0.0
   delay(state, 1.0)
   vessel.control.activate_next_stage()
   delay(state, 2.0)
   vessel.control.throttle = 1.0
   vessel.control.activate_next_stage()
  
   print('atmospheric flight guidance finished')   