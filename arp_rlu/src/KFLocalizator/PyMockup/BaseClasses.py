from numpy import *
import math

def betweenMinusPiAndPlusPi(angle):
  angle = betweenZeroAndTwoPi( angle )
  if angle > pi:
    angle = angle - 2 * pi
  if angle < -pi:
    angle = angle + 2 * pi
  return angle


def betweenZeroAndTwoPi(angle):
  return fmod( fmod(angle, 2 * pi) + 4. * pi, 2. * pi);

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
    self.tt    = array( () )
    self.theta = array( () )
    self.range = array( () )
    
class PointCloud:
  def __init__(self, N = 0):
    self.points = zeros( (2,N))
    self.tt = zeros( (N) )
    
class Object:
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
    filtscan = copy(scan)
    infIndex = (self.N -1)/2
    supIndex = self.N -1 - (self.N -1)/2
    for j in range(int(scan.shape[1])):
      if j-infIndex < 0:
        continue
      if j+supIndex > int(scan.shape[1]) - 1:
        continue
      v = list(scan[1,(j-infIndex):(j+supIndex+1)])
      filtscan[1,j] = self.getMedian(v)
    return filtscan
  
def computePointCloud(scan, tt, xx, yy, hh):
  pc = PointCloud()
  pc.points = zeros( (2, scan.theta.shape[0]) )
  pc.points[0,0:] = xx[0:] + scan.range[0:] * cos(scan.theta[0:] + hh[0:])
  pc.points[1,0:] = yy[0:] + scan.range[0:] * sin(scan.theta[0:] + hh[0:])
  pc.tt = array(tt)
  return pc

