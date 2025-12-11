import sys 
sys.path.append("..") 
from cse import *
import time
from poliastro.bodies import Earth
from poliastro.twobody import Orbit
from astropy import units as u

r0 = (7000000, 0, -12124000) # m
v0 = (2667.9, 0, 4621.0) # m/s
dt = 3600

start = time.perf_counter()
r, v, x = conic_state_extrapolation(r0, v0, dt)
end = time.perf_counter()
print('r: {}, v: {}'.format(r, v))
print(f"Function took {end - start:.6f} seconds\n")

 # Create an Orbit object
start = time.perf_counter()
orb = Orbit.from_vectors(Earth, r0 * u.m, v0 * u.m/u.s)
# Propagate the orbit for a given time
dt = 1 * u.hour
future_orb = orb.propagate(dt)
end = time.perf_counter()
print(f"Future position: {future_orb.r}")
print(f"Future velocity: {future_orb.v}")
print(f"Function took {end - start:.6f} seconds")