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
