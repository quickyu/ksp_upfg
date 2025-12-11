import sys 
sys.path.append("..") 
import numpy as np
import krpc

from utilities import * 

client = krpc.connect(name='Test')
vessel = client.space_center.active_vessel
print(vessel.name)

normal = target_normal(vessel, np.radians(0.0), np.radians(0.0))
v = rodrigues_rotation(np.array([1.0, 0.0, 0.0]), -normal, -np.pi/2)
print(v)