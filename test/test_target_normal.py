import sys 
sys.path.append("..") 
import math
import numpy as np
import unittest

from utilities import * 

class TestTargetNormal(unittest.TestCase):
   def __init__(self, methodName="runTest"):
      super(TestTargetNormal, self).__init__(methodName)   

   def test(self):   
      test_data = [
         {'inc': 0.0, 'raan': 0.0, 'normal': [0.0, 0.0, 1.0]},
         {'inc': 180.0, 'raan': 0.0, 'normal': [0.0, 0.0, -1.0]},
         {'inc': 90.0, 'raan': 0.0, 'normal': [0.0, -1.0, 0.0]},
         {'inc': 90.0, 'raan': 90.0, 'normal': [1.0, 0.0, 0.0]},
         {'inc': 51.6, 'raan': 0.0, 'normal': [0.000000, -0.783693, 0.621148]},
         {'inc': 67.4172, 'raan': 20.9101, 'normal': [0.329537, -0.862517, 0.384018]},
         {'inc': 171.1286, 'raan': 311.8234, 'normal': [-0.114924, -0.102838, -0.988037]},
         {'inc': 131.7589, 'raan': 216.4014, 'normal': [-0.442678, 0.600403, -0.665998]},
         {'inc': 107.7585, 'raan': 254.9061, 'normal': [-0.919495, 0.247993, -0.305006]},
         {'inc': 28.0834, 'raan': 7.4104, 'normal': [0.060716, -0.466824, 0.882264]},
         {'inc': 28.0790, 'raan': 349.1675, 'normal': [-0.088460, -0.462302, 0.882299]}
      ]

      for sample in test_data:
         normal = target_normal(math.radians(sample['inc']), math.radians(sample['raan']))
         normal = normalize_vector(normal)
         print(f'normal: {normal}')
         self.assertTrue(np.allclose(normal[[0, 2, 1]], sample['normal']))

if __name__ == '__main__':
   unittest.main()    