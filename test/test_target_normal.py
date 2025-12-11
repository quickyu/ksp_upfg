import sys 
sys.path.append("..") 
from utilities import * 

client = krpc.connect(name='Test')
vessel = client.space_center.active_vessel
print(vessel.name)

normal = target_normal(vessel, np.radians(60), np.radians(0.0))
normal = normalize_vector(normal)
print(f'normal: {normal}')