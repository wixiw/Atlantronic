from numpy import *
import math

class OdoVelocity:
  def __init__(self):
    self.vx = 0.
    self.vy = 0.
    self.va = 0.

class Scan:
  def __init__(self):
    self.tbeg  = 0.
    self.tend  = 0.
    self.tt    = array( () )
    self.theta = array( () )
    self.range = array( () )
    
class Object:
  def __init_(self):
    self.xCenter = 0.0
    self.yCenter = 0.0
    self.radius  = 0.0
    self.tBeg   = None
    self.tEnd   = None

class Estimate:
  def __init__(self):
    self.xRobot = None
    self.yRobot = None
    self.hRobot = None
    self.velXRobot = None
    self.velYRobot = None
    self.velHRobot = None
    self.covariance = None
  
  
class RingBuffer:
  def __init__(self, size):
    self.data = [None for i in xrange(size)]
  def append(self, x):
    self.data.pop(0)
    self.data.append(x)
  def getAll(self):
    return self.data
  def getOldest(self):
    return self.data[0]
  def getNewest(self):
    return self.data[-1:]

def interp1d( t, tp, yp ):
    y = interp(t, tp, yp)
    if len(tp) < 2:
        return y
    for i in range(len(t)):
        if t[i] < tp[0]:
            y[i] = yp[0] - (tp[0]-t[i])*(yp[1]-yp[0])/(tp[1]-tp[0])
        if t[i] > tp[-1]:
            y[i] = yp[-1] + (t[i]-tp[-1])*(yp[-1]-yp[-2])/(tp[-1]-tp[-2])
    return y

def interpMatrix( t, tp, yp):
  y = []
  for k in range(len(t)):
    y.append( zeros( yp[0].shape ) )
  for i in range(int(yp[0].shape[0])):
    for j in range(int(yp[0].shape[1])):
      yp_ = []
      for k in range(len(tp)):
        yp_.append(yp[k][i,j])
      for k in range(len(t)):
        y[k][i,j] = interp1d([t[k]], tp, yp_)
  return y
