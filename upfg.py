from krpc import *
from krpc.services.spacecenter import *
import numpy as np
import math
from collections import OrderedDict

import settings
from vessel_state import *
from utilities import *

g0 = 9.80655
mu = 398600441800000.0
equatorial_radius = 6378137.0

class Struct:
   pass

def norm(x):
   return np.linalg.norm(x)

def unit(x):
   x = np.asarray(x)
   if norm(x) == 0:
      return x
   else:
      return x / norm(x)

def cross(x, y):
   return np.cross(x, y)

def dot(x, y):
   return np.vdot(x, y)

def vang(x, y):
   x = unit(x)
   y = unit(y)
   return np.rad2deg(np.acos(np.clip(dot(x, y), -1, 1)))

def setup_upfg_target() -> Struct:
   if settings.mission['altitude'] < settings.mission['periapsis'] or settings.mission['altitude'] > settings.mission['apoapsis']: 
      settings.mission["altitude"]  = settings.mission["periapsis"]
       
   pe = settings.mission['periapsis'] + equatorial_radius
   ap = settings.mission['apoapsis'] + equatorial_radius
   target_altitude = settings.mission['altitude'] + equatorial_radius
   sma = (pe + ap) / 2
   vpe = math.sqrt(mu * (2/pe - 1/sma))
   srm = pe * vpe
   target_velocity = math.sqrt(mu * (2/target_altitude - 1/sma))
   flight_path_angle = math.acos(srm / (target_velocity * target_altitude)) # radians

   target = Struct()
   target.radius = target_altitude

   normal = target_normal(settings.mission['inclination'], settings.mission['LAN'])
   normal = -normal[[0, 2, 1]] # UPFG compatible direction
   target.normal = normal

   target.angle = math.degrees(flight_path_angle)
   target.velocity = target_velocity

   print(f'UPFG target: radius={target_altitude:.2f}, normal={target.normal}, angle={target.angle:.2f}, velocity={target.velocity:.2f}')
      
   return target

def upfg_initial(state:VesselState, target:Struct) -> Struct:
   position = state.position_eci()
   dest_r = rodrigues_rotation(unit(position), -target.normal, math.radians(20.0))
   dest_r = dest_r * target.radius
    
   velocity = state.velocity_eci()
   vgo = target.velocity * unit(np.cross(-target.normal, dest_r)) - velocity

   internal = Struct()

   cser = Struct()
   cser.dtcp = 0.
   cser.xcp = 0.
   cser.a = 0.
   cser.d = 0.
   cser.e = 0.
   internal.cser = cser
   internal.rbias = [0., 0., 0.]
   internal.rd = dest_r
   internal.rgrav = np.multiply(np.multiply(-(mu / 2), position), 1 / np.linalg.norm(position) ** 3)
   internal.tb = 0.
   internal.time = state.universal_time()
   internal.tgo = 0.
   internal.v = velocity
   internal.vgo = vgo

   return internal
    
def upfg_control(vehicle:list, target:Struct, previous:Struct, state:VesselState) -> list:
   gamma = math.radians(target.angle)
   iy = np.asarray(target.normal)
   rdval = target.radius
   vdval = target.velocity
   t = state.universal_time()
   m = state.mass() 
   r = state.position_eci() 
   r = np.array([r[0], r[2], r[1]])
   v = state.velocity_eci() 
   v = np.array([v[0], v[2], v[1]])
   cser = previous.cser
   rbias = np.asarray(previous.rbias)
   rd = np.asarray(previous.rd)
   rgrav = np.asarray(previous.rgrav)
   tp = previous.time
   vprev = np.asarray(previous.v)
   vgo = np.asarray(previous.vgo)

   n = len(vehicle)
   md = list()
   ve = list()
   f_t = list()
   a_t = list()
   tu = list()
   tb = list()
   #if n > 1 and state.thrust() < 0.01:
   #   stage_controller(vehicle)
   #   return upfg(vehicle, target, previous)

   for i in range(n):
      f_t.append(vehicle[i].fT)
      ve.append(vehicle[i].ve)
      md.append(f_t[i] / ve[i])
      a_t.append(f_t[i] / vehicle[i].m0)
      tu.append(ve[i] / a_t[i])
      tb.append(vehicle[i].maxT)

   dt = t - tp
   dvsensed = v - vprev
   vgo = vgo - dvsensed
   # vgo1 = vgo
   tb[0] = tb[0] - previous.tb
   f_t[0] = state.thrust()
   a_t[0] = f_t[0] / m
   tu[0] = ve[0] / a_t[0]
   l_ = 0
   li_ = list()

   for i in range(n - 1):
      li_.append(ve[i] * np.log(tu[i] / (tu[i] - tb[i])))
      l_ += li_[i]
      if l_ > norm(vgo):
         vehicle.remove(vehicle[-1])
         print('We have more than what we need')
         return upfg_control(vehicle, target, previous, state)
   li_.append(norm(vgo) - l_)

   tgoi = list()

   for i in range(n):
      tb[i] = tu[i] * (1 - np.exp(-li_[i] / ve[i]))
      if i == 0:
         tgoi.append(tb[i])
      else:
         tgoi.append(tgoi[i - 1] + tb[i])

   # l1 = li[0]
   tgo = tgoi[n - 1]
   if tgo > 5:
      theta_max = math.radians(60.0)
   else:
      theta_max = math.radians(1.0)

   l_ = 0
   s_ = 0
   j_ = 0
   q_ = 0
   p_ = 0
   h_ = 0
   ji_ = list()
   si_ = list()
   qi_ = list()
   pi_ = list()
   tgoi1 = 0

   for i in range(n):
      if i > 0:
         tgoi1 = tgoi[i - 1]
      ji_.append(tu[i] * li_[i] - ve[i] * tb[i])
      si_.append(tb[i] * li_[i] - ji_[i])
      qi_.append(si_[i] * (tu[i] + tgoi1) - 0.5 * ve[i] * tb[i] ** 2)
      pi_.append(qi_[i] * (tu[i] + tgoi1) - 0.5 * ve[i] * tb[i] ** 2 * (tb[i] / 3 + tgoi1))

      ji_[i] += li_[i] * tgoi1
      si_[i] += l_ * tb[i]
      qi_[i] += j_ * tb[i]
      pi_[i] += h_ * tb[i]

      l_ += li_[i]
      j_ += ji_[i]
      s_ += si_[i]
      q_ += qi_[i]
      p_ += pi_[i]
      h_ = j_ * tgoi[i] - q_

   lamb = unit(vgo)
   # rgrav1 = rgrav
   if previous.tgo != 0:
      rgrav = (tgo / previous.tgo) ** 2 * rgrav
   rgo = rd - (r + v * tgo + rgrav)
   # rgo1 = rgo
   iz = unit(cross(rd, iy))
   # iz1 = iz
   rgoxy = rgo - dot(iz, rgo) * iz
   rgoz = (s_ - dot(lamb, rgoxy)) / dot(lamb, iz)
   rgo = rgoxy + rgoz * iz + rbias
   lambdade = q_ - s_ * j_ / l_
   lambdadot = (rgo - s_ * lamb) / lambdade
   if (norm(lambdadot) * j_ / l_) > theta_max:
      lambdadotmag = theta_max / (j_ / l_)
      lambdadot = unit(lambdadot) * lambdadotmag
      rgo = s_ * lamb + lambdade * lambdadot
   i_f = unit(lamb - lambdadot * j_ / l_)
   phi = np.arccos(dot(i_f, lamb))
   phidot = -phi * l_ / j_
   vthrust = (l_ - 0.5 * l_ * phi ** 2 - j_ * phi *
            phidot - 0.5 * h_ * phidot ** 2) * lamb
   vthrust = vthrust - (l_ * phi + j_ * phidot) * unit(lambdadot)
   rthrust = (s_ - 0.5 * s_ * phi ** 2 - q_ * phi *
            phidot - 0.5 * p_ * phidot ** 2) * lamb
   rthrust = rthrust - (s_ * phi + q_ * phidot) * unit(lambdadot)
   vbias = vgo - vthrust
   rbias = rgo - rthrust

   i_f = [i_f[0], i_f[2], i_f[1]]

   rc1 = r - 0.1 * rthrust - vthrust * tgo / 30
   vc1 = v + rthrust * 1.2 / tgo - 0.1 * vthrust
   [rc2, vc2, cser] = cse_routine(rc1, vc1, tgo, cser)
   vgrav = vc2 - vc1
   rgrav = rc2 - rc1 - vc1 * tgo

   rp = r + v * tgo + rgrav + rthrust
   rp = rp - dot(rp, iy) * iy
   rd = rdval * unit(rp)
   ix = unit(rd)
   iz = cross(ix, iy)
   m1 = np.transpose([ix, iy, iz])
   m2 = np.transpose([math.sin(gamma), 0, math.cos(gamma)])
   mx = np.matmul(m1, m2).transpose()
   vd = vdval * mx
   vgo = vd - v - vgrav + vbias

   previous.cser = cser
   previous.rbias = rbias
   previous.rd = rd
   previous.rgrav = rgrav
   previous.tb = previous.tb + dt
   previous.time = t
   previous.tgo = tgo
   previous.v = v
   previous.vgo = vgo

   guidance = Struct()
   guidance.i_f = i_f
   guidance.tgo = tgo

   return previous, guidance

def cse_routine(r0, v0, dt, last):
   r0 = np.asarray(r0)
   v0 = np.asarray(v0)
   if last.dtcp == 0:
      dtcp = dt
   else:
      dtcp = last.dtcp

   xcp = last.xcp
   x = xcp
   a = last.a
   d = last.d
   e = last.e

   kmax = 10
   imax = 10

   if dt > 0:
      f0 = 1
   else:
      f0 = -1

   n = 0
   r0m = norm(r0)

   f1 = f0 * np.sqrt(r0m / mu)
   f2 = 1 / f1
   f3 = f2 / r0m
   f4 = f1 * r0m
   f5 = f0 / np.sqrt(r0m)
   f6 = f0 * np.sqrt(r0m)

   ir0 = r0 / r0m
   v0s = f1 * v0
   sigma0s = dot(ir0, v0s)
   b0 = dot(v0s, v0s) - 1
   alphas = 1 - b0

   xguess = f5 * x
   xlast = f5 * xcp
   xmin = 0
   dts = f3 * dt
   dtlast = f3 * dtcp
   dtmin = 0

   xmax = np.divide(2 * np.pi, np.sqrt(np.abs(alphas)))

   if alphas > 0:
      dtmax = xmax / alphas
      xp = xmax
      ps = dtmax
      while dts >= ps:
         n = n + 1
         dts = dts - ps
         dtlast = dtlast - ps
         xguess = xguess - xp
         xlast = xlast - xp
   else:
      [dtmax, _, _, _] = k_t_t_i(xmax, sigma0s, alphas, kmax)
      if dtmax < dts:
         while dtmax >= dts:
               dtmin = dtmax
               xmin = xmax
               xmax = np.multiply(2, xmax)
               [dtmax, _, _, _] = k_t_t_i(xmax, sigma0s, alphas, kmax)

   if xmin >= xguess or xguess >= xmax:
      xguess = 0.5 * (xmin + xmax)

   [dtguess, _, _, _] = k_t_t_i(xguess, sigma0s, alphas, kmax)

   if dts < dtguess:
      if xguess < xlast < xmax:
         if dtguess < dtlast < dtmax:
               xmax = xlast
               dtmax = dtlast
   else:
      if xmin < xlast < xguess:
         if dtmin < dtlast < dtguess:
               xmin = xlast
               dtmin = dtlast

   [xguess, dtguess, a, d, e] = k_i_l(
      imax, dts, xguess, dtguess, xmin, dtmin, xmax, dtmax,
      sigma0s, alphas, kmax, a, d, e)

   rs = 1 + 2 * (b0 * a + sigma0s * d * e)
   b4 = 1 / rs

   if n > 0:
      # noinspection PyUnboundLocalVariable
      xc = f6 * (xguess + n * xp)
      # noinspection PyUnboundLocalVariable
      dtc = f4 * (dtguess + n * ps)
   else:
      xc = f6 * xguess
      dtc = f4 * dtguess

   last.dtcp = dtc
   last.xcp = xc
   last.a = a
   last.d = d
   last.e = e

   f = 1 - 2 * a
   gs = 2 * (d * e + sigma0s * a)
   fts = -2 * b4 * d * e
   gt = 1 - 2 * b4 * a

   r = r0m * (f * ir0 + gs * v0s)
   v = f2 * (fts * ir0 + gt * v0s)

   return [r, v, last]

def k_t_t_i(xarg, s0s, a, kmax):
   u1 = uss(xarg, a, kmax)
   zs = 2 * u1
   e = 1 - 0.5 * a * zs ** 2
   w = np.sqrt(max(0.5 + e / 2, 0))
   d = w * zs
   a = d ** 2
   b = 2 * (e + s0s * d)
   q = qcf(w)
   t = d * (b + a * q)

   return [t, a, d, e]

def uss(xarg, a, kmax):
   du1 = np.divide(xarg, 4)
   u1 = du1
   f7 = -a * du1 ** 2
   k = 3
   while k < kmax:
      du1 = f7 * du1 / (k * (k - 1))
      u1old = u1
      u1 = u1 + du1
      if u1 == u1old:
         break
      k += 2
   return u1

def qcf(w):
   if w < 1:
      xq = 21.04 - 13.04 * w
   elif w < 4.625:
      xq = (5 / 3) * (2 * w + 5)
   elif w < 13.846:
      xq = (10 / 7) * (w + 12)
   elif w < 44:
      xq = 0.5 * (w + 60)
   elif w < 100:
      xq = 0.25 * (w + 164)
   else:
      xq = 70

   b = 0
   y = (w - 1) / (w + 1)
   j = np.floor(xq)
   b = y / (1 + (j - 1) / (j + 2) * (1 - b))
   while j > 2:
      j = j - 1
      b = y / (1 + (j - 1) / (j + 2) * (1 - b))

   q = 1 / w ** 2 * (1 + (2 - b / 2) / (3 * w * (w + 1)))
   return q

def k_i_l(imax, dts, xguess, dtguess, xmin, dtmin, xmax, dtmax,
         s0s, alphas, kmax, a, d, e):
   i = 1
   while i < imax:
      dterror = dts - dtguess

      if abs(dterror) < 0.0000001:
         break

      [dxs, xmin, dtmin, xmax, dtmax] = si(
         dterror, xguess, dtguess, xmin, dtmin, xmax, dtmax)
      xold = xguess
      xguess = xguess + dxs

      if xguess == xold:
         break

      dtold = dtguess
      [dtguess, a, d, e] = k_t_t_i(xguess, s0s, alphas, kmax)

      if dtguess == dtold:
         break

      i += 1

   return [xguess, dtguess, a, d, e]

def si(dterror, xguess, dtguess, xmin, dtmin, xmax, dtmax):
   etp = 0.0000001
   dtminp = dtguess - dtmin
   dtmaxp = dtguess - dtmax
   if abs(dtminp) < etp or abs(dtmaxp) < etp:
      dxs = 0
   else:
      if dterror < 0:
         dxs = (xguess - xmax) * (dterror / dtmaxp)
         if (xguess + dxs) <= xmin:
               dxs = (xguess - xmin) * (dterror / dtminp)
         xmax = xguess
         dtmax = dtguess
      else:
         dxs = (xguess - xmin) * (dterror / dtminp)
         if (xguess + dxs) >= xmax:
               dxs = (xguess - xmax) * (dterror / dtmaxp)
         xmin = xguess
         dtmin = dtguess

   return [dxs, xmin, dtmin, xmax, dtmax]

def analyze_vehicle(vessel:Vessel):
   stage = list()
   for part in vessel.parts.all:
      if part.engine is not None:
         stage.append(part.engine.part.decouple_stage)

   stage = list(OrderedDict.fromkeys(stage))
   m0 = list()
   m1 = list()
   f_t = list()
   ve = list()
   a_t = list()
   tu = list()
   l1 = list()
   tb = list()
   max_throttle = list()
   min_throttle = list()
   mass = 0

   for i in range(len(stage)):
      part_list = vessel.parts.in_decouple_stage(stage[i])
      fuel_name = list()
      thrust = 0
      max_thrust = 0
      min_thrust = 0
      flow_rate = 0
      isp = 0
      for part in part_list:
         mass += part.mass
         if part.engine is not None:
               thrust += part.engine.max_vacuum_thrust
               isp += part.engine.vacuum_specific_impulse * g0
               flow_rate += thrust / isp
               for fuel in part.engine.propellants:
                  fuel_name.append(fuel.name)
               prev_limit = part.engine.thrust_limit
               part.engine.thrust_limit = 0
               min_thrust += part.engine.available_thrust
               part.engine.thrust_limit = prev_limit
               max_thrust += part.engine.available_thrust

      resources_list = vessel.resources_in_decouple_stage(
         stage[i] + 1, False)
      fuel_mass = 0
      fuel_name = list(OrderedDict.fromkeys(fuel_name))
      for fuel in fuel_name:
         for resource in resources_list.all:
            if resource.name == fuel:
               fuel_mass += resource.amount * resource.density 

      m0.append(mass)
      m1.append(fuel_mass)
      f_t.append(thrust)
      ve.append(thrust / flow_rate)
      a_t.append(thrust / mass)
      tb.append(fuel_mass / flow_rate)
      tu.append(ve[i] * mass / thrust)
      l1.append(ve[i] * np.log(tu[i] / (tu[i] - tb[i])))
      min_throttle.append(min_thrust / max_thrust)
      max_throttle.append(max_thrust / max_thrust)

   m0.reverse()
   m1.reverse()
   f_t.reverse()
   ve.reverse()
   a_t.reverse()
   tu.reverse()
   l1.reverse()
   tb.reverse()
   min_throttle.reverse()
   max_throttle.reverse()
   vehicle = list()
   for i in range(len(stage)):
      stages = Struct()
      stages.m0 = m0[i]
      stages.fT = f_t[i]
      stages.ve = ve[i]
      stages.l1 = l1[i]
      stages.maxThrottle = max_throttle[i]
      stages.minThrottle = min_throttle[i]
      stages.maxT = tb[i]
      vehicle.append(stages)

   return vehicle

