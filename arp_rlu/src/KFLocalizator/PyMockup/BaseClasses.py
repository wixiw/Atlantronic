import numpy as np
import math
from abc import ABCMeta

def betweenMinusPiAndPlusPi(angle):
  angle = betweenZeroAndTwoPi( angle )
  if angle > np.pi:
    angle = angle - 2 * np.pi
  if angle < -np.pi:
    angle = angle + 2 * np.pi
  return angle


def betweenZeroAndTwoPi(angle):
  return np.fmod( np.fmod(angle, 2 * np.pi) + 4. * np.pi, 2. * np.pi);

class OdoVelocity:
  def __init__(self):
    self.vx = 0.
    self.vy = 0.
    self.vh = 0.

class Scan:
  def __init__(self):
    self.tsync = 0.
    self.tbeg  = 0.
    self.tend  = 0.
    self.tt    = np.array( () )
    self.theta = np.array( () )
    self.range = np.array( () )
  def copy(self):
    retval = Scan()
    retval.tsync = self.tsync
    retval.tbeg = self.tbeg
    retval.tend = self.tend
    retval.tt = np.copy(self.tt)
    retval.theta = np.copy(self.theta)
    retval.range = np.copy(self.range)
    return retval
    
class PointCloud:
  def __init__(self, N = 0):
    self.tt = np.zeros( (N) )
    self.points = np.zeros( (2,N))
  def fromScan(self, scan, tt, xx, yy, hh):
    n = scan.theta.shape[0]
    if type(tt) != type([]):
      tt = [tt] * n
    if type(xx) != type([]):
      xx = [xx] * n
    if type(yy) != type([]):
      yy = [yy] * n
    if type(hh) != type([]):
      hh = [hh] * n
    if len(tt) != n:
      raise TypeError("tt should be scalars or list with the same length than scan.theta. len(tt)=%d" % len(tt))
    if len(xx) != n:
      raise TypeError("xx should be scalars or list with the same length than scan.theta. len(xx)=%d" % len(xx))
    if len(yy) != n:
      raise TypeError("yy should be scalars or list with the same length than scan.theta. len(yy)=%d" % len(yy))
    if len(hh) != n:
      raise TypeError("hh should be scalars or list with the same length than scan.theta. len(hh)=%d" % len(hh))
    self.points = np.zeros( (2, scan.theta.shape[0]) )
    self.points[0,0:] = xx[0:] + scan.range[0:] * np.cos(scan.theta[0:] + hh[0:])
    self.points[1,0:] = yy[0:] + scan.range[0:] * np.sin(scan.theta[0:] + hh[0:])
    self.tt = np.array(tt)
  def cleanUp(self):
    self.tt  = self.tt[np.nonzero(self.points[1,0:])]
    self.points = np.vstack( (self.points[0, np.nonzero(self.points[1,0:])], self.points[1, np.nonzero(self.points[1,0:])]) )
    
    
class Object:
  __metaclass__ = ABCMeta
  
class Circle(Object):
  def __init__(self):
    self.xCenter  = 0.0
    self.yCenter  = 0.0
    self.radius   = 0.0
    self.thetaBeg = None
    self.thetaEnd = None
    self.timeBeg  = None
    self.timeEnd  = None
    self.used     = False
  def __str__(self):
    return "xCenter=%f  yCenter=%f r=%f thetaBeg=%f thetaEnd=%f timeBeg=%f timeEnd=%f" % (self.xCenter, self.yCenter, self.radius, self.thetaBeg, self.thetaEnd, self.timeBeg, self.timeEnd)
    
class Segment(Object):
  def __init__(self):
    self.A = np.zeros( (2,1) )
    self.B = np.zeros( (2,1) )
  def __str__(self):
    return "A.x=%f  A.y=%f r=%f B.x=%f B.y=%f" % (self.A[0,0], self.A[1,0], self.B[0,0], self.B[1,0])
      


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
    self.N = size
    self.data = [None for i in xrange(self.N)]
  def append(self, x):
    self.data.pop(0)
    self.data.append(x)
  def clear(self):
    self.data = [None for i in xrange(self.N)]
  def getAll(self):
    return self.data
  def getAllNoNone(self):
    return filter(lambda x: x != None ,self.data)
  def getOldest(self):
    return self.data[0]
  def getNewest(self):
    return self.data[-1]

def interp1d( t, tp, yp ):
    y = np.interp(t, tp, yp)
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
    y.append( np.zeros( yp[0].shape ) )
  for i in range(int(yp[0].shape[0])):
    for j in range(int(yp[0].shape[1])):
      yp_ = []
      for k in range(len(tp)):
        yp_.append(yp[k][i,j])
      for k in range(len(t)):
        y[k][i,j] = interp1d([t[k]], tp, yp_)
  return y

def pca(x):
  means = np.mean(x, 1)
  cov = np.cov( x - means.reshape((2,1)) ) 
  values, vectors = np.linalg.eig( cov )
  return means, np.sqrt(values), vectors

class MedianFilter:
  def __init__(self, N):
    self.N = N
  def getMedian(self, v):
    if len(v) == 1:
      return v[0]
    noChange = False
    while not noChange:
      noChange = True
      for j in range(len(v)-1):
        if v[j] > v[j+1]:
          tmp = v[j+1]
          v[j+1] = v[j]
          v[j] = tmp
          noChange = False
    return v[(len(v)-1)/2]
  def compute(self, scan):
    filtscan = scan.copy()
    infIndex = (self.N -1)/2
    supIndex = self.N -1 - (self.N -1)/2
    for j in range(int(scan.range.shape[0])):
      if j-infIndex < 0:
        continue
      if j+supIndex > int(scan.range.shape[0]) - 1:
        continue
      v = list(scan.range[(j-infIndex):(j+supIndex+1)])
      filtscan.range[j] = self.getMedian(v)
    return filtscan
  

