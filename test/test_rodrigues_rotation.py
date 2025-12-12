import sys 
sys.path.append("..") 
import unittest
import numpy as np
import quaternion

from utilities import * 

def quaternion_rotation(v:np.ndarray, k:np.ndarray, theta:float) -> np.ndarray:
   k = k / np.linalg.norm(k)
   q = np.quaternion(np.cos(theta/2), np.sin(theta/2)*k[0], np.sin(theta/2)*k[1], np.sin(theta/2)*k[2])
   return quaternion.rotate_vectors(q, v)

class TestRodriguesRotation(unittest.TestCase):
   def __init__(self, methodName="runTest"):
      super(TestRodriguesRotation, self).__init__(methodName)

   def test(self):   
      for i in range(100):
         v = np.random.rand(3)
         k = np.random.rand(3)
         theta = np.random.rand() * 2 * np.pi

         v1 = rodrigues_rotation(v, k, theta)
         v2 = quaternion_rotation(v, k, theta)
         print(f'v1: {v1}, v2: {v2}')

         self.assertTrue(np.allclose(v1, v2))

if __name__ == '__main__':
   unittest.main()   