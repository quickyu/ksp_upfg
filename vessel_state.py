from krpc import *
from krpc.services.spacecenter import *
import numpy as np   

class VesselState:
   def __init__(self, vessel:Vessel):
      self.vessel_ = vessel   
      self.client_ = self.vessel_._client
      self.space_center_ = self.client_.space_center

      self.ut_ = self.client_.add_stream(getattr, self.client_.space_center, 'ut')
      self.mass_ = self.client_.add_stream(getattr, vessel, 'mass')
      self.dry_mass_ = self.client_.add_stream(getattr, vessel, 'dry_mass')
      self.position_  = self.client_.add_stream(vessel.position, self.vessel_.orbit.body.reference_frame)
      self.velocity_ = self.client_.add_stream(vessel.velocity, self.vessel_.orbit.body.reference_frame)
      self.thrust_ = self.client_.add_stream(getattr, vessel, 'thrust') 
      self.isp_ = self.client_.add_stream(getattr, vessel, 'specific_impulse')

   def universal_time(self) -> float:
      return self.ut_()
   
   def mass(self) -> float:
      return self.mass_()
   
   def dry_mass(self) -> float:
      return self.dry_mass_()
   
   def position(self) -> np.ndarray:
      return np.array(self.position_())
   
   def velocity(self) -> np.ndarray:
      return np.array(self.velocity_())
   
   def thrust(self) -> float:
      return self.thrust_()   
   
   def specific_impulse(self) -> float:
      return self.isp_()
