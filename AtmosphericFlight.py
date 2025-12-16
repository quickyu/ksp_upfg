import krpc
from krpc.services.spacecenter import *
import time
import math
import numpy as np

import settings
from utilities import *

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

   client.space_center.warp_to(liftoff_time - 30)

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
  
            orbit_normal = target_normal_ECEF(vessel, np.radians(settings.mission['inclination']), np.radians(settings.mission['LAN']))

            line = client.drawing.add_direction_from_com(orbit_normal.tolist(), earth.reference_frame, 200)
            line.color = (0, 1, 0)
            line.thickness = 1

      time.sleep(0.01)
      current_time = int(client.space_center.ut)

   if indicator:
      client.drawing.clear()   
      
def atmospheric_flight_control(vessel:Vessel, upfg_target:dict):
   client = vessel._client
   
   ref_frame = client.space_center.ReferenceFrame.create_hybrid(
      position=vessel.orbit.body.non_rotating_reference_frame,
      rotation=vessel.surface_reference_frame)

   flight_info = vessel.flight()

   ut = client.add_stream(getattr, client.space_center, 'ut')
   velocity = client.add_stream(vessel.velocity, ref_frame)
   mean_altitude = client.add_stream(getattr, flight_info, 'mean_altitude')
   pitch = client.add_stream(getattr, flight_info, 'pitch')
   roll = client.add_stream(getattr, flight_info, 'roll')
   heading = client.add_stream(getattr, flight_info, 'heading')
   angle_of_attack = client.add_stream(getattr, flight_info, 'angle_of_attack')
   
   target_azimuth = launch_azimuth(vessel, upfg_target['velocity'], settings.mission['inclination'], upfg_target['angle'])
   print(f'Target azimuth: {target_azimuth:.2f}')
   initial_azi = math.degrees(initial_azimuth(vessel))
   print(f'Initial azimuth: {initial_azi:.2f}')

   vessel.control.throttle = 1.0
   vessel.control.sas = False
   vessel.control.rcs = False

   vessel.auto_pilot.target_pitch_and_heading(90.0, initial_azi)
   vessel.auto_pilot.engage()

   vessel.control.activate_next_stage() # main engine ignition
   time.sleep(5)
   vessel.control.activate_next_stage() # booster ignition
   time.sleep(0.1)
   vessel.control.activate_next_stage() # lift off

   liftoff_time = ut()
   print(f'Liftoff at {liftoff_time:.2f}')

   state = FlightState.takeoff

   booster_separated = False  
   stage1_separated = False
   finished = False

   start_pitch_manuever = False
   pitch_done = False   

   while not finished:
      flight_time = ut() - liftoff_time
      
      if not booster_separated and flight_time >= settings.booster_burn_time:
         print('Booster separation')
         vessel.control.activate_next_stage()
         booster_separated = True

      if not stage1_separated and flight_time >= settings.first_stage_burn_time:
         print('Stage 1 separation')
         stage1_separated = True
         vessel.control.throttle = 0.0
         time.sleep(0.1)
         vessel.control.activate_next_stage()
         time.sleep(1)
         vessel.control.throttle = 1.0
         vessel.control.activate_next_stage()
         
      match state:
         case FlightState.takeoff:
            #print('State: takeoff')
            if mean_altitude() > settings.roll_altitude:
               state = FlightState.roll_maneuver
         case FlightState.roll_maneuver:
            #print('State: roll_maneuver')
            azi = math.degrees(initial_azimuth(vessel))
            #print(f'roll_maneuver: azi {azi:.2f}')
            if (math.fabs(azi - target_azimuth) < 1.0):
               state = FlightState.pitch_maneuver
               pitch_angle = 90.0
            else:   
               vessel.auto_pilot.target_pitch_and_heading(90, target_azimuth)       
         case FlightState.pitch_maneuver: 
            #print('State: pitch_maneuver')
            if not start_pitch_manuever:
               if velocity()[0] > settings.pitch_velocity:
                  start_pitch_manuever = True
                  pitch_start_time = flight_time

                  pitch_angle -= settings.pitch_rate
                  vessel.auto_pilot.target_pitch_and_heading(pitch_angle, target_azimuth)   
            else:      
               if flight_time - pitch_start_time > 1.0:
                  pitch_start_time = flight_time

                  if not pitch_done:
                     #print(f'pitch: {pitch():.2f}')

                     if math.fabs(pitch_angle - 90.0 + settings.pitch_maneuver_angle) < 0.5:
                        pitch_done = True
                     else:
                        pitch_angle -= 0.5
                        vessel.auto_pilot.target_pitch_and_heading(pitch_angle, target_azimuth)   
                  else:      
                     #print(f'aoa: {angle_of_attack():.2f}')
                     if math.fabs(angle_of_attack()) < 0.5:
                        state = FlightState.gravity_turn
                        vessel.auto_pilot.target_roll = 0.0
         case FlightState.gravity_turn:            
            diff = pitch() - angle_of_attack()
            vessel.auto_pilot.target_pitch_and_heading(diff, target_azimuth) 
            if math.fabs(pitch() - settings.target_picth) < 1.0:
               vessel.auto_pilot.target_pitch_and_heading(settings.target_picth, target_azimuth)
               state = FlightState.wait_stage1_separated
         case FlightState.wait_stage1_separated:   
            if stage1_separated:   
               finished = True

      time.sleep(0.01)  

   print('atmospheric_flight_control finished')   