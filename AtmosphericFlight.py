from krpc.services.spacecenter import *
import time
import numpy as np

from utilities import *
from vessel_state import VesselState

class FlightState:
   wait_ignition = 0,
   wait_srb_ignition = 1,  
   wait_liftoff = 2,
   in_flight = 3,

def read_guidance_file(file_path:str) -> list[list]:
   data_list = [] 
   with open(file_path, mode='r') as csv_file:
      csv_reader = csv.reader(csv_file, delimiter=',')

      line_num = int(0)             
      for row in csv_reader:
         try:
            row_data = [float(x) for x in row]
            data_list.append(row_data)
         except ValueError:
            data_list.append(row) 
         line_num += 1

      if line_num != 15:
         raise Exception('Incorrect number of lines.')  
         
   return data_list      

def interp(guidance:list[list], time:float, start_index:int) -> tuple[float, float, int]:
   timestamp = guidance[0]
   azi_angles = guidance[1]
   pitch_angles = guidance[2]  

   if time < timestamp[0]:
      azi = azi_angles[0]
      pitch = pitch_angles[0]
      index  = 0
   elif time > timestamp[-1]:  
      azi = azi_angles[-1]
      pitch = pitch_angles[-1]
      index = len(timestamp) - 1
   else:
      index = -1
      for i in range(start_index, len(timestamp)):
         if time >= timestamp[i] and time < timestamp[i + 1]:  
            index = i  
            break

      x0 = timestamp[index]  
      x1 = timestamp[index + 1]

      y0 = pitch_angles[index] 
      y1 = pitch_angles[index + 1]
      pitch = (y0 * (x1 - time) + y1 * (time - x0)) / (x1 - x0)

      y0 = (azi_angles[index] + 180.0) % 360.0 - 180.0
      y1 = (azi_angles[index + 1] + 180.0) % 360.0 - 180.0
      azi = (y0 * (x1 - time) + y1 * (time - x0)) / (x1 - x0)

   return azi, pitch, index

def ignition_offset(guidance:list[list]) -> int:
   length = len(guidance[0])
   for i in range(length):
      if guidance[13][i] == 'Release Clamp':  
         return -int(guidance[0][i])
   return 0  

def draw_normal_indicator(vessel:Vessel, target_inc, target_lan):      
   client = vessel._client
   client.drawing.clear()

   position = cartesian_position(vessel, vessel.orbit.body.reference_frame) # ECEF position
   zenith_direction = normalize_vector(np.array(position))
   earth = vessel.orbit.body
   line = client.drawing.add_direction_from_com(zenith_direction, earth.reference_frame, 200)
   line.color = (1, 0, 0)
   line.thickness = 1

   orbit_normal = target_normal_ECEF(vessel, target_inc, target_lan)
   line = client.drawing.add_direction_from_com(orbit_normal.tolist(), earth.reference_frame, 200)
   line.color = (0, 1, 0)
   line.thickness = 1

def atmospheric_flight_control(vessel:Vessel, upfg_target, state:VesselState, ignition_time:int, liftoff_time:int, guidance:list[list], mission_config:dict):
   flight_state = FlightState.wait_ignition  

   srb_ignition_time = liftoff_time - 1
   srb_burn_time = int(mission_config['booster_burn_time'])

   target_azimuth = 0.0
   initial_azi = 0.0

   booster_separated = False  

   start_index = int(0)

   state.ut_.start()
   
   while True:
      with state.ut_.condition:
         state.ut_.condition.wait()

      current_time = state.universal_time()

      if flight_state == FlightState.wait_ignition:
         draw_normal_indicator(vessel, mission_config['inclination'], mission_config['LAN'])

         if int(current_time) == ignition_time:
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

            vessel._client.drawing.clear()
            flight_state = FlightState.wait_srb_ignition
      elif flight_state == FlightState.wait_srb_ignition:    
         if int(current_time) == srb_ignition_time:
            vessel.control.activate_next_stage()
            flight_state = FlightState.wait_liftoff
      elif flight_state == FlightState.wait_liftoff: 
         if int(current_time) == liftoff_time:
            vessel.control.activate_next_stage()
            flight_state = FlightState.in_flight
      elif flight_state == FlightState.in_flight:     
         if int(current_time) == srb_ignition_time + srb_burn_time:
            if not booster_separated:
               print('Booster separation')
               vessel.control.activate_next_stage()
               booster_separated = True

         mission_seconds = current_time - float(ignition_time)
         print(f'Mission seconds: {mission_seconds:.2f}')

         if mission_seconds <= guidance[0][-1]:
            aoa = state.angle_of_attack()
            azi_ctrl, pitch_ctrl, start_index = interp(guidance, mission_seconds, start_index)
            print(f'Azi: {azi_ctrl:.2f}, Pitch: {pitch_ctrl:.2f}, AOA: {aoa:.2f}')
            vessel.auto_pilot.target_pitch = pitch_ctrl
            vessel.auto_pilot.target_roll = 0.0
            vessel.auto_pilot.target_heading = azi_ctrl
         else:
            break
         
   print('Stage 1 separation')
   vessel.control.throttle = 0.0
   delay(state, 1.0)
   vessel.control.activate_next_stage()
   delay(state, 2.0)
   vessel.control.throttle = 1.0
   vessel.control.activate_next_stage()
  
   print('atmospheric flight guidance finished')   