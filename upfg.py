from krpc import *
from krpc.services.spacecenter import *
import numpy as np
import math

import settings
from vessel_state import *
from utilities import *

vehicle = None
upfg_target = None
upfg_internal = None

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

def setup_target_normal(normal:np.ndarray):
   upfg_target['normal'] = normal 

def upfg_initial(vessel:Vessel, target:dict) -> dict:
   ref_frame = vessel.orbit.body.reference_frame
   normal = target_normal(vessel, settings.mission['inclination'], settings.mission['LAN'])

   position = np.array(vessel.position(ref_frame))
   dest_r = rodrigues_rotation(position, -normal, -np.radians(20))
   dest_r = normalize_vector(dest_r) * target['radius']
  
   velocity = np.array(vessel.velocity(ref_frame))
   vgo = target['velocity'] * normalize_vector(np.cross(normal, dest_r)) - velocity

   upfg_internal =  {
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

def initialize_vehicle_for_upfg(state:VesselState):
   mass_total = state.mass()
   mass_dry = state.dry_mass()
   mass_fuel = mass_total - mass_dry
   max_t = mass_fuel / (state.thrust() / state.specific_impulse() / 9.81)
   
   vehicle = {
      'mass_total': mass_total,
      'mass_dry': mass_dry,
      'mass_fuel': mass_fuel,
      'max_t': max_t,
      'engine_isp': state.specific_impulse(),
      'engine_thrust': state.thrust()
      }
def upfg(current_state:VesselState) -> dict:
   #block0
   gamma	= upfg_target['angle']
   iy = upfg_target['normal']
   rdval = upfg_target['radius']
   vdval = upfg_target['velocity']

   current_time = current_state.universal_time()
   mass = current_state.mass()
   pos = current_state.position()
   vel = current_state.velocity()

   cse_x0 = upfg_internal['cse_x0']
   rbias = upfg_internal['rbias']
   rd = upfg_internal['rd']
   rgrav = upfg_internal['rgrav']
   prev_time = upfg_internal['time']   
   prev_vel = upfg_internal['v']
   vgo = upfg_internal['vgo']

   #block1
   ve = vehicle['engine_isp'] * 9.81
   ft = vehicle['engine_thrust']
   tb = vehicle['max_t']

   #block2
   dt = current_time - prev_time

   dv = vel - upfg_internal['v']
   vgo = upfg_internal['vgo'] - dv
   vgo1 = vgo

   tb = tb - upfg_internal['tb']

   #block3 
   tu = ve / (ft / mass)
   Li = np.linalg.norm(vgo)
   tb = tu * (1 - math.exp(-Li / ve))
   tgo = tb
   tgoi1 = tb

   #block4 
   L = Li
   J = tu * Li - ve * tb + L * tgoi1
   S = -J + tb * Li

