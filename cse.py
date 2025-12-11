import numpy as np
import math

MU = 398600441800000.0

def conic_state_extrapolation(r0, v0, dt, x0=0.0, tol=5e-9):
   rscale = np.linalg.norm(r0)
   vscale = np.sqrt(MU / rscale)
   r0s = r0 / rscale
   v0s = v0 / vscale 
   dts = dt * vscale / rscale
   v2s = np.square(np.linalg.norm(v0)) * rscale / MU
   alpha = 2 - v2s
   armd1 = v2s - 1
   rvr0s = np.dot(r0, v0) / np.sqrt(MU * rscale)

   x = dts * np.abs(alpha) if x0 == 0.0 else x0
   x2 = x * x
   z = alpha * x2
   sz, cz = snc(z)
   x2cz = x2 * cz

   ratio = 1.0
   while np.abs(ratio) > tol:
      f = x + rvr0s * x2cz + armd1 * x * x2 * sz - dts
      df = x * rvr0s * (1 - z * sz) + armd1 * x2cz + 1.
      ratio = f / df
      x = x - ratio
      x2 = x * x
      z = alpha * x2
      sz, cz = snc(z)  
      x2cz = x2 * cz

   lf = 1 - x2cz
   lg = dts - x2 * x * sz
   r1 = lf * r0s + lg * v0s
   ir1 = 1.0 / np.linalg.norm(r1)   
   lfdot = ir1 * x * (z * sz - 1)
   lgdot = 1 - x2cz * ir1

   v1 = lfdot * r0s + lgdot * v0s

   return r1 * rscale, v1 * vscale, x
def snc(z):
   az = abs(z)
   if az < 1e-4:
      return (1 - z * (0.05 - z / 840)) / 6, 0.5 - z * (1 - z / 30) / 24
   else:
      saz = math.sqrt(az)
      if z > 0:
         return (saz - math.sin(saz)) / (saz * az), (1 - math.cos(saz)) / az
      else:
         x = math.exp(saz)
         return (0.5 * (x - 1 / x) - saz) / (saz * az), (0.5 * (x + 1 / x) - 1) / az

