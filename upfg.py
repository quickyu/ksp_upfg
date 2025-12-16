from krpc import *
from krpc.services.spacecenter import *
import numpy as np
import math

import settings
from vessel_state import *
from utilities import *

def setup_upfg_target(vessel:Vessel) -> dict:
   if settings.mission['altitude'] < settings.mission['periapsis'] or settings.mission['altitude'] > settings.mission['apoapsis']: 
      settings.mission["altitude"]  = settings.mission["periapsis"]
       
   earth = vessel.orbit.body
   pe = settings.mission['periapsis'] + earth.equatorial_radius
   ap = settings.mission['apoapsis'] + earth.equatorial_radius
   target_altitude = settings.mission['altitude'] + earth.equatorial_radius
   sma = (pe + ap) / 2
   vpe = math.sqrt(earth.gravitational_parameter * (2/pe - 1/sma))
   srm = pe * vpe
   target_velocity = math.sqrt(earth.gravitational_parameter * (2/target_altitude - 1/sma))
   flight_path_angle = math.acos(srm / (target_velocity * target_altitude)) # radians

   upfg_target = {'radius': target_altitude, 'velocity': target_velocity, 'angle': flight_path_angle}
   return upfg_target

def setup_target_normal(vessel:Vessel, inc:float, lan:float, target:dict):
   # UPFG compatible direction.
   # UPFG requires it to point opposite to a direction of vector of angular momentum for the orbit.
   target['normal'] = -target_normal_ECEF(vessel, inc, lan) 
    
def upfg_initial(vessel:Vessel, target:dict) -> dict:
   ref_frame = vessel.orbit.body.reference_frame
   normal = target_normal_ECEF(vessel, settings.mission['inclination'], settings.mission['LAN'])

   position = np.array(vessel.position(ref_frame))
   dest_r = rodrigues_rotation(position, normal, np.radians(-20.0))
   dest_r = normalize_vector(dest_r) * target['radius']
  
   velocity = np.array(vessel.velocity(ref_frame))
   vgo = target['velocity'] * normalize_vector(np.cross(dest_r, normal)) - velocity

   return {
      'cse_x0': 0.0, 
      'rbias': np.array([0.0, 0.0, 0.0]), 
      'rd': dest_r, 
      'rgrav': -vessel.orbit.body.gravitational_parameter/2 * position / np.linalg.norm(position)**3,
      'tb': 0.0,
      'time': vessel._client.space_center.ut,
      'tgo': 0.0,
      'v': velocity,
      'vgo': vgo
      }

def upfg_control(current_state:VesselState, target:dict, internal:dict) -> dict:
   #block0
   gamma	= target['angle']
   iy = target['normal']
   rdval = target['radius']
   vdval = target['velocity']

   current_time = current_state.universal_time()
   mass = current_state.mass()
   pos = current_state.position()
   vel = current_state.velocity()

   cse_x0 = internal['cse_x0']
   rbias = internal['rbias']
   rd = internal['rd']
   rgrav = internal['rgrav']
   prev_time = internal['time']   
   prev_vel = internal['v']
   vgo = internal['vgo']

   #block1
   ve = current_state.specific_impulse() * 9.81
   ft = current_state.thrust()
   tb = settings.stage2_burn_time

   #block2
   dt = current_time - prev_time
   dv = vel - internal['v']
   vgo = internal['vgo'] - dv
   #tb = tb - internal['tb']

   #block3 
   tu = ve / (ft / mass)
   L1 = np.linalg.norm(vgo)
   tb = tu * (1 - math.exp(-L1 / ve))
   tgo = tb

   #block4 
   L = L1
   J = tu * L1 - ve * tb + L * tgo
   S = -J + tb * L

