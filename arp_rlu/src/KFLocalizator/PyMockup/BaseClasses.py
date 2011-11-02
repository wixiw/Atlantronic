import numpy as np
import math
from abc import ABCMeta
import logging

from Scan import Scan
from PointCloud import PointCloud

from BaseMethods import *

class OdoVelocity:
  def __init__(self):
    self.vx = 0.
    self.vy = 0.
    self.vh = 0.
  
    
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
  def __init__(self, x = None, y = None, h = None, l = None):
    self.x = x
    self.y = y
    self.h = h
    self.l = l
    if x != None and y != None and h != None and l != None:
      self.A = np.array( [[x],[y]]) + 0.5 * l * np.array([[math.cos(h - np.pi/2.)],[math.sin(h - np.pi/2.)]])
      self.B = np.array( [[x],[y]]) + 0.5 * l * np.array([[math.cos(h + np.pi/2.)],[math.sin(h + np.pi/2.)]])
  def __str__(self):
    return "A.x=%f  A.y=%f r=%f B.x=%f B.y=%f" % (self.A[0,0], self.A[1,0], self.B[0,0], self.B[1,0])
 
class FoundObject(Object):
  def __init__(self):
    self.x = None     
    self.y = None     
    self.beta = None
    self.hb = None
    self.detectionTime = None
    self.detectionTheta = None
    self.detectionRange = None
    self.nbPoints = 0
    self.used = False
  def __str__(self):
    return "x=%s  y=%s hb=%s beta=%s detectionTime=%s detectionTheta=%s detectionRange=%s nbPoints=%d" % (str(self.x), str(self.y), str(self.hb), str(self.beta), str(self.detectionTime), str(self.detectionTheta), str(self.detectionRange), self.nbPoints)
    
    


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


  

