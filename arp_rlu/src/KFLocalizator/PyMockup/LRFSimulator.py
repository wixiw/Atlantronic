import numpy as np
import math
import random

from BaseClasses import Scan
from BaseClasses import Circle
from BaseClasses import Segment

class LRFSimulator:
  def __init__(self):
    self.sigma = 0.
    self.objects = []
    
  def circleIntersection(self, x, y, theta, obj):
    # M is Robot center and O is object center
    MO = np.array( [ [obj.xCenter - x], [obj.yCenter - y] ] )
    
    # laser direction vector
    u  = np.array( [[np.cos(theta)], [np.sin(theta)]] )
    
    # P is O projected on (M, theta)
    if np.vdot(u, MO) < 0.:
      range = float('inf')
      intersection = None
      return range, intersection
    else:
      PO = MO - u * np.vdot(u, MO)
      if np.linalg.norm(PO) < obj.radius:
        delta = math.sqrt( obj.radius * obj.radius - np.linalg.norm(PO) * np.linalg.norm(PO) )
        range = np.vdot(u, MO) - delta
        intersection = np.array( [ [x], [y] ] ) + u * range
        return range, intersection
      else:
        range = float('inf')
        intersection = None
        return range, intersection
      
  def segmentIntersection(self, x, y, theta, sgmt):
    range = float('inf')
    intersection = None
    
    M = np.array( [[x],[y]])
    D = np.dot( np.linalg.pinv( np.hstack((sgmt.B - sgmt.A, -np.array([[math.cos(theta)],[math.sin(theta)]]) )) ), (M - sgmt.A))
    
    if D[1,0] > 0:
      if D[0,0] >= 0 and D[0,0] <= 1:
        range = D[1,0]
        intersection = sgmt.A + D[0,0] * (sgmt.B - sgmt.A)
    return range, intersection
    
  def rayTracer(self, x, y, a):
    range = float('inf')
    intersection = None
    for obj in self.objects:
      r = float('inf')
      inter = None
      if isinstance(obj, Circle):
        r, inter = self.circleIntersection(x, y, a, obj)
      elif isinstance(obj, Segment):
        r, inter = self.segmentIntersection(x, y, a, obj)
      if r < range:
        range = r
        intersection = inter        
    return range, intersection
    
  def computeScan(self, tt, xx, yy, hh):
    # xx numpy array size (1,N) for xRobot
    # yy numpy array size (1,N) for yRobot
    # aa numpy array size (1,N) for headingRobot
    # tt numpy array size (1,N) for time
    
    s = Scan()
    s.theta = np.arange( -2.*np.pi*340./1024., -2.*np.pi*340./1024. + 2.*np.pi*681./1024., 2.*np.pi/1024.)
    
    if len(s.theta) > len(tt):
      raise ValueError("tt vector is too small (<", len(s.theta), ") : len(tt)=" + str(len(tt)))
    if len(xx) != len(tt):
      raise ValueError('xx and tt do not have same length')
    if len(yy) != len(tt):
      raise ValueError('yy and tt do not have same length')
    if len(hh) != len(tt):
      raise ValueError('aa and tt do not have same length')
    
    ideb = len(tt) - len(s.theta)
    s.tbeg  = tt[ideb]
    s.range = np.zeros( (len(s.theta)) )
    s.tt    = np.zeros( (len(s.theta)) )
    s.tsync = tt[ideb] - 44. * 0.1 / 1024.
    
    for i in range(len(s.theta)):
      t = tt[ideb+i]
      x = xx[ideb+i]
      y = yy[ideb+i]
      h = hh[ideb+i]
      s.tend = t
      s.tt[i] = t
      r, inter = self.rayTracer(x, y, h + s.theta[i])
      if inter != None:
        s.range[i] = r + random.normalvariate(0.0, self.sigma)
    
    return s
    
    
    
    
