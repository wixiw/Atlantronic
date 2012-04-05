# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import random
import logging

import numpy as np
from BaseMethods import *

def gen(params):
  log = logging.getLogger('generate_mouvement')
  
  xo = random.uniform( params["minXPos"], params["maxXPos"])
  yo = random.uniform( params["minYPos"], params["maxYPos"])
  ho = random.uniform( 0., 2. * np.pi)
  log.info("x(0):%f en m", xo)
  log.info("y(0):%f en m", yo)
  log.info("h(0):%f en deg", ho*180.0/np.pi)
  
  vxo = random.uniform( -params["maxLinVel"], params["maxLinVel"])
  vyo = random.uniform( -params["maxLinVel"], params["maxLinVel"])
  vho = random.uniform( -params["maxRotVel"], params["maxRotVel"])
  log.info("vx(0):%f en m/s", vxo)
  log.info("vy(0):%f en m/s", vyo)
  log.info("vh(0):%f en deg/s", vho*180.0/np.pi)
  
  axo = random.uniform( -params["maxLinAcc"], params["maxLinAcc"])
  ayo = random.uniform( -params["maxLinAcc"], params["maxLinAcc"])
  aho = random.uniform( -params["maxRotAcc"], params["maxRotAcc"])
  log.info("ax(t):%f en m/s**2",axo)
  log.info("ay(t):%f en m/s**2",ayo)
  log.info("ah(t):%f en deg/s**2",aho*180.0/np.pi)
  
#  dt = 0.1 / 1000.
  dt = 0.1 / 1024.
  td = 1./dt
  tt = np.arange( 0., params["duration"], dt)
  
  ax = np.ones_like(tt) * axo
  ay = np.ones_like(tt) * ayo
  ah = np.ones_like(tt) * aho
  
  vx = np.ones_like(tt) * vxo
  vy = np.ones_like(tt) * vyo
  vh = np.ones_like(tt) * vho
  for i in range(1, len(tt)):
    vx[i] = vx[i-1] + dt * ax[i-1]
    vy[i] = vy[i-1] + dt * ay[i-1]
    vh[i] = vh[i-1] + dt * ah[i-1]
  vx = np.clip(vx, -params["maxLinVel"], params["maxLinVel"])
  vy = np.clip(vy, -params["maxLinVel"], params["maxLinVel"])
  vh = np.clip(vh, -params["maxRotVel"], params["maxRotVel"])
  
  xx = np.ones_like(tt) * xo
  yy = np.ones_like(tt) * yo
  hh = np.ones_like(tt) * ho
  for i in range(1, len(tt)):
    xx[i] = xx[i-1] + dt * vx[i-1]
    yy[i] = yy[i-1] + dt * vy[i-1]
    hh[i] = hh[i-1] + dt * vh[i-1]
  xx = np.clip(xx, params["minXPos"], params["maxXPos"])
  yy = np.clip(yy, params["minYPos"], params["maxYPos"])
  
  # clipping incfluence propagation
  for i in range(1, len(tt)):
    vx[i-1] = (xx[i] - xx[i-1] ) * td
    vy[i-1] = (yy[i] - yy[i-1] ) * td
    vh[i-1] = (hh[i] - hh[i-1] ) * td
    
  for i in range(1, len(tt)):
    ax[i-1] = (vx[i] - vx[i-1] ) * td
    ay[i-1] = (vy[i] - vy[i-1] ) * td
    ah[i-1] = (vh[i] - vh[i-1] ) * td
  
  return tt, xx, yy, hh, vx, vy, vh, ax, ay, ah