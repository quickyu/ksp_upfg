import sys 
sys.path.append("..") 
import numpy as np

from utilities import * 

v = rodrigues_rotation(np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), -np.pi/2)
print(v)