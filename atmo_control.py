import krpc
from krpc.services.spacecenter import *
import time
import math
import numpy as np

import settings
from utilities import *

def wait_for_launch(vessel:Vessel, liftoff_time:int, indicator=False):
   client = vessel._client

   refresh_counter = 0
   current_time = int(client.space_center.ut)

   while current_time < liftoff_time:
      if indicator:
         refresh_counter += 1
         if refresh_counter == 10:
            refresh_counter = 0

            client.drawing.clear()

            position = cartesian_position(vessel)
            ls_normal = normalize_vector(np.array(position))
            earth = vessel.orbit.body
            line = client.drawing.add_direction_from_com(ls_normal, earth.reference_frame, 60)
            line.color = (1, 0, 0)
            line.thickness = 1
  
            orbit_normal = target_normal(vessel, np.radians(settings.mission['inclination']), np.radians(settings.mission['LAN']))
            line = client.drawing.add_direction_from_com(tuple(orbit_normal.tolist()), earth.reference_frame, 60)
            line.color = (0, 1, 0)
            line.thickness = 1

      time.sleep(0.01)
      current_time = int(client.space_center.ut)

   if indicator:
      client.drawing.clear()   
      
def atmospheric_control(vessel:Vessel, upfg_target:dict):
   client = vessel._client

   ref_frame = client.space_center.ReferenceFrame.create_hybrid(
      position=vessel.orbit.body.non_rotating_reference_frame,
      rotation=vessel.surface_reference_frame)
   
   azimuth = launch_azimuth(vessel, upfg_target['velocity'], settings.mission['inclination'], upfg_target['angle'])
   print('Azimuth: {}'.format(azimuth))
   azi_init = math.degrees(initial_azimuth(vessel))

   vessel.control.throttle = 1.0
   vessel.control.sas = False
   vessel.control.rcs = False
   vessel.auto_pilot.target_pitch_and_heading(90, azi_init)
   vessel.auto_pilot.engage()

   state = 0
   stage1_separated = False
   fairing_separated = False

   liftoff_time = client.space_center.ut

   vessel.control.activate_next_stage() # ignition
   time.sleep(3)
   vessel.control.activate_next_stage() # lift off
   
   while True:
      flight_time = client.space_center.ut - liftoff_time

      if not stage1_separated and flight_time >= settings.controls['meco_time']:
         print('MECO')
         stage1_separated = True
         vessel.control.throttle = 0.0
         time.sleep(1)
         print('Booster separation')
         vessel.control.activate_next_stage()
         time.sleep(1)
         vessel.control.throttle = 1.0
         vessel.control.activate_next_stage()

      if stage1_separated and not fairing_separated:
         mean_altitude = vessel.flight().mean_altitude
         if mean_altitude >= settings.controls['fairing_separation_altitude']:
            print('Fairing separation')
            vessel.control.activate_next_stage()
            fairing_separated = True

      if state == 0:
         if (flight_time >= settings.controls['roll_over_time']):
            vessel.auto_pilot.target_pitch_and_heading(90, azimuth)
            state = 1
      if state == 1:
         vertical_speed = np.linalg.norm(vessel.flight(ref_frame).vertical_speed)
         if vertical_speed >= settings.controls['pitch_over_velocity']:
            state = 2
      elif state == 2:
         pitch = 90 - settings.controls['pitch_over_angle']
         vessel.auto_pilot.target_pitch_and_heading(pitch, azimuth)

         while True:
            current_pitch = vessel.flight().pitch
            #print('current pitch: {}'.format(current_pitch))
            if math.fabs(current_pitch - pitch) < 1.0:
               break

         while True:
            aoa = vessel.flight().angle_of_attack
            #print('aoa: {}'.format(aoa))
            if math.fabs(aoa) < 0.1:
               break

         print('Gravity turn')      
         state = 3
      elif state == 3:
         aoa = vessel.flight().angle_of_attack
         p = vessel.flight().pitch - aoa
         vessel.auto_pilot.target_pitch_and_heading(p, azimuth) 

         if math.fabs(vessel.flight().pitch - settings.controls['min_pitch_angle']) < 0.1:
            vessel.auto_pilot.target_pitch_and_heading(settings.controls['min_pitch_angle'], azimuth)
            state = 4
      elif state == 4:   
         if stage1_separated and fairing_separated:
            vessel.auto_pilot.target_pitch_and_heading(20, azimuth)
            return

      time.sleep(0.1)   