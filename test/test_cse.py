import sys 
sys.path.append("..") 

import unittest
from poliastro.bodies import Earth
from poliastro.twobody import Orbit
from astropy import units as u

from cse import *

class TestCSE(unittest.TestCase):
   def __init__(self, methodName="runTest"):
      super(TestCSE, self).__init__(methodName)

   def test(self):
      position = [
         [7000000.0, -12124000.0, 0.0], 
         [6678137.0, 0.0, 0.0], 
         [7000000.0, 0.0, 0.0],
         [26560000.0, 0.0, 0.0],
      ] # m
      
      velocity = [
         [2667.9, 4621.0, 0.0], 
         [0.0, 0.0, 7612.2],
         [0.0, 1000.0, 7546.049],
         [0.0, 500.0, 2965.000]
      ] # m/s
      
      dt = [600, 1800, 3600, 7200, 14400, 28800, 57600, 86400]

      for row in range(4):
         print(f'num={row}')
         r0 = position[row]
         v0 = velocity[row]

         for t in dt:
            print(f't = {t}')
            r1, v1, x = conic_state_extrapolation(r0, v0, t)
            print(f'r1 = {r1}, v1 = {v1}')

            orb = Orbit.from_vectors(Earth, r0 * u.m, v0 * u.m/u.s)
            delta_t = t * u.s
            future_orb = orb.propagate(delta_t)
            r2 = future_orb.r.to(u.m).value.tolist()
            v2 = future_orb.v.to(u.m / u.s).value.tolist()
            print(f'r2 = {r2}, v2 = {v2}\n')

            self.assertTrue(np.allclose(r1, r2))
            self.assertTrue(np.allclose(v1, v2))

         print('\n')   

if __name__ == '__main__':
   unittest.main()         