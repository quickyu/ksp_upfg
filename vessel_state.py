from krpc import *
from krpc.services.spacecenter import *
import numpy as np   

class VesselState:
   def __init__(self, vessel:Vessel):
      client = vessel._client
    
      self.ut_ = client.add_stream(getattr, client.space_center, 'ut')

      self.mass_ = client.add_stream(getattr, vessel, 'mass')

      self.dry_mass_ = client.add_stream(getattr, vessel, 'dry_mass')

      self.position_ecef_  = client.add_stream(vessel.position, vessel.orbit.body.reference_frame)

      self.velocity_ecef_ = client.add_stream(vessel.velocity, vessel.orbit.body.reference_frame)

      self.position_eci_ = client.add_stream(vessel.position, vessel.orbit.body.non_rotating_reference_frame)

      self.velocity_eci_ = client.add_stream(vessel.velocity, vessel.orbit.body.non_rotating_reference_frame)

      self.thrust_ = client.add_stream(getattr, vessel, 'thrust') 

      self.available_thrust_ = client.add_stream(getattr, vessel, 'available_thrust')

      self.isp_ = client.add_stream(getattr, vessel, 'specific_impulse')

      self.direction_eci_ = client.add_stream(vessel.direction, vessel.orbit.body.non_rotating_reference_frame)
      
      self.speed_eci_ = client.add_stream(getattr, vessel.flight(vessel.orbit.body.non_rotating_reference_frame), 'speed')

      ref_frame = client.space_center.ReferenceFrame.create_hybrid(position=vessel.orbit.body.non_rotating_reference_frame,
                                                               rotation=vessel.surface_reference_frame)
      
      self.vertical_speed_ = client.add_stream(getattr, vessel.flight(ref_frame), 'vertical_speed')

      self.mean_altitude_ = client.add_stream(getattr, vessel.flight(), 'mean_altitude')

      self.pitch_ = client.add_stream(getattr, vessel.flight(), 'pitch')

      self.roll_ = client.add_stream(getattr, vessel.flight(), 'roll')

      self.heading_ = client.add_stream(getattr, vessel.flight(), 'heading')

      self.angle_of_attack_ = client.add_stream(getattr, vessel.flight(), 'angle_of_attack')

   def universal_time(self) -> float:
      return self.ut_()
   
   def mass(self) -> float:
      return self.mass_()
   
   def dry_mass(self) -> float:
      return self.dry_mass_()
   
   def position_ecef(self) -> np.ndarray:
      return np.array(self.position_ecef_())
   
   def velocity_ecef(self) -> np.ndarray:
      return np.array(self.velocity_ecef_())
   
   def position_eci(self) -> np.ndarray:
      return np.array(self.position_eci_())
   
   def velocity_eci(self) -> np.ndarray:
      return np.array(self.velocity_eci_())
   
   def thrust(self) -> float:
      return self.thrust_()   
   
   def available_thrust(self) -> float:
      return self.available_thrust_()
   
   def specific_impulse(self) -> float:
      return self.isp_()
   
   def direction_eci(self) -> np.ndarray:
      return np.array(self.direction_eci_())
   
   def speed_eci(self) -> float:
      return self.speed_eci_()
   
   def vertical_speed(self) -> float:
      return self.vertical_speed_()
   
   def pitch(self) -> float:
      return self.pitch_()
   
   def roll(self) -> float:
      return self.roll_()
   
   def heading(self) -> float:
      return self.heading_()
   
   def angle_of_attack(self) -> float:
      return self.angle_of_attack_()
   
   def mean_altitude(self) -> float:
      return self.mean_altitude_()
