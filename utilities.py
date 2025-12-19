from krpc import *
from krpc.services.spacecenter import *
import numpy as np
import math
import time

import settings
from vessel_state import VesselState

def initial_azimuth(vessel:Vessel) -> float:
   client = vessel._client
   ori = client.space_center.transform_direction((0, 0, 1), vessel.reference_frame, vessel.surface_reference_frame)
   azi = angle_between_vectors(np.array([0, 1, 0]), np.array(ori))
   if ori[2] < 0: 
      azi = -azi
   return math.degrees(azi)

def launch_azimuth(vessel:Vessel, target_vel, target_inc, flight_path_angle) -> float:
   long, lat = geo_position(vessel)
   binertial = math.cos(math.radians(target_inc)) / math.cos(math.radians(lat))
   if binertial < -1:
      binertial = -1
   if binertial > 1: 
      binertial = 1
   binertial = math.asin(binertial)

   earth = vessel.orbit.body
   vbody = (2 * math.pi * earth.equatorial_radius / earth.rotational_period ) / math.cos(math.radians(lat))
   vorbit = target_vel * math.cos(math.radians(flight_path_angle))
   vrotx = vorbit * math.sin(binertial) - vbody
   vroty = vorbit * math.cos(binertial)
   azi = math.atan2(vroty, vrotx)
   azi = math.degrees(azi)

   if settings.mission['direction'].upper() == 'NORTH':
      return 90.0 - azi
   elif settings.mission['direction'].upper() == 'SOUTH':
      return 90.0 + azi
   else:
      raise Exception('Unknown launch direction.')

def angle_between_vectors(a:np.ndarray, b:np.ndarray) -> float:
   if np.linalg.norm(a) == 0.0 or np.linalg.norm(b) == 0.0:
      return 0.0
    
   dot_product = np.dot(a, b)
   norm_a = np.linalg.norm(a)
   norm_b = np.linalg.norm(b)

   cos_theta = dot_product / (norm_a * norm_b)
   cos_theta = np.clip(cos_theta, -1.0, 1.0)

   return np.arccos(cos_theta)

def geo_position(vessel:Vessel) -> tuple[float, float]:
   earth = vessel.orbit.body
   ecef_reference_frame = earth.reference_frame
   position_ecef = vessel.position(ecef_reference_frame) 
   long = earth.longitude_at_position(position_ecef, ecef_reference_frame)
   lat = earth.latitude_at_position(position_ecef, ecef_reference_frame)
   return long, lat

def cartesian_position(vessel:Vessel, reference_frame:ReferenceFrame) -> tuple[float, float, float]:
   return vessel.position(reference_frame) 

def rodrigues_rotation(v:np.ndarray, k:np.ndarray, theta:float) -> np.ndarray:
   """
      Using the Rodrigues' formula to rotate a vector v around axis k by θ radians

      Parameters:
      v: The original vector, shape (3,) or (N,3)
      k: The rotation axis, shape (3,) (must be a unit vector)
      θ: The rotation angle (in radians)

      Returns:
      The rotated vector, shape (3,) or (N,3)
   """
   k = k / np.linalg.norm(k)
    
   cos_theta = np.cos(theta)
   sin_theta = np.sin(theta)
    
   term1 = v * cos_theta                   # v*cosθ
   term2 = np.cross(k, v) * sin_theta      # (k×v)*sinθ
   term3 = k * np.dot(k, v) * (1-cos_theta)  # k*(k·v)*(1-cosθ)
    
   return term1 + term2 + term3

def vector_exclude(v1:np.ndarray, v2:np.ndarray) -> np.ndarray:
   """
      This is a vector, v2 with all of v1 excluded from it. In other words, the projection of v2 onto the plane that is normal to v1.
   """
   v1 = v1 / np.linalg.norm(v1)
   v2 = v2 - np.dot(v2, v1) * v1
   return v2

def normalize_vector(v:np.ndarray) -> np.ndarray:
   norm = np.linalg.norm(v, axis=-1, keepdims=True)
   return v / norm if norm.any() != 0 else v

def ascending_node_vector(vessel:Vessel, orbit_inclination:float, dir:str) -> np.ndarray:
   long, lat = geo_position(vessel)

   b = np.tan(np.pi/2 - orbit_inclination) * np.tan(np.radians(lat))
   b = np.arcsin(np.fmin(np.fmax(-1., b), 1.))

   position = cartesian_position(vessel, vessel.orbit.body.reference_frame)
   long_vector = vector_exclude(np.array([0., 1., 0.]), -np.array(position))
   long_vector = normalize_vector(long_vector)

   if dir.upper() == 'NORTH':
      v = rodrigues_rotation(long_vector, np.array([0., 1., 0.]), -b)
   elif dir.upper() == 'SOUTH':
      v = rodrigues_rotation(long_vector, np.array([0., 1., 0.]), np.pi+b)
   else:
      raise Exception('Unknown launch direction.')
   
   return v

def orbit_intercept_time(vessel:Vessel, dir:str, inclination:float, lan:float) -> float:
   client = vessel._client

   inclination = math.radians(inclination)
   lan = math.radians(lan)

   if dir.upper() == 'NEAREST':
      time_n = orbit_intercept_time(client, 'NORTH', inclination, lan)
      time_s = orbit_intercept_time(client, 'SOUTH', inclination, lan)
      if time_s < time_n:
         return time_s
      else:
         return time_n
   else:
      current_node = ascending_node_vector(vessel, inclination, dir)  

      prime_vector = rodrigues_rotation(np.array([1., 0., 0.]), np.array([0., 1., 0.]), -lan)
      target_ascending_node = client.space_center.transform_direction(
            tuple(prime_vector.tolist()), 
            vessel.orbit.body.non_rotating_reference_frame,
            vessel.orbit.body.reference_frame) 
      target_ascending_node = np.array(target_ascending_node)
      
      node_delta = angle_between_vectors(current_node, target_ascending_node)
      
      delta_dir = np.dot(np.array([0., 1., 0.]), np.cross(target_ascending_node, current_node))
      if delta_dir < 0:
         node_delta = 2*np.pi - node_delta

      return vessel.orbit.body.rotational_period * node_delta / (2 * np.pi)
   
def target_normal(inc:float, lan:float) -> np.ndarray:
   inc = math.radians(inc)
   lan = math.radians(lan)

   high_point = rodrigues_rotation(np.array([1., 0., 0.]), np.array([0., 1., 0.]), np.pi/2 - lan)
   rot_axis = normalize_vector(high_point[[2, 1, 0]]*np.array([-1., 1., 1.]))

   return rodrigues_rotation(high_point, rot_axis, np.pi/2 - inc) # ksp left hand ECI coordinate system 

def target_normal_ECEF(vessel:Vessel, inc:float, lan:float) -> np.ndarray:
   client = vessel._client
   return np.array(client.space_center.transform_direction(
                     target_normal(inc, lan).tolist(), 
                     vessel.orbit.body.non_rotating_reference_frame,
                     vessel.orbit.body.reference_frame))

def draw_vector(vessel:Vessel, vector:np.ndarray, color:tuple, ref_frame:ReferenceFrame, length:float):
   client = vessel._client

   vector = vector / np.linalg.norm(vector)

   line = client.drawing.add_direction_from_com(vector.tolist(), ref_frame, length)
   line.color = color
   line.thickness = 1

def clear_lines(vessel:Vessel):
   client = vessel._client
   client.drawing.clear()

def delay(state:VesselState, t:float):
   t -= 0.02
   ts = state.universal_time()
   elapse = 0.0
   while elapse < t:
      time.sleep(0.001)
      elapse = state.universal_time() - ts   
