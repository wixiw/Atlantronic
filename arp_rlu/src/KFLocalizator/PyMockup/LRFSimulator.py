from numpy import *
import math
import random

from BaseClasses import Scan
from BaseClasses import Object

class LRFSimulator:
  def __init__(self):
    self.sigma = 0.
    self.objects = []
    
  def circleIntersection(self, x, y, theta, obj):
    # M is Robot center and O is object center
    MO = array( [ [obj.xCenter - x], [obj.yCenter - y] ] )
    # print "MO:", MO
    
    # laser direction vector
    u  = array( [[cos(theta)], [sin(theta)]] )
    # print "u:",u
    # print "vdot(u, MO):",vdot(u, MO)
    
    # P is O projected on (M, theta)
    if vdot(u, MO) < 0.:
      range = float('inf')
      intersection = None
      return range, intersection
    else:
      PO = MO - u * vdot(u, MO)
      # print "PO:",PO
      if linalg.norm(PO) < obj.radius:
        delta = math.sqrt( obj.radius * obj.radius - linalg.norm(PO) * linalg.norm(PO) )
        range = vdot(u, MO) - delta
        intersection = array( [ [x], [y] ] ) + u * range
        return range, intersection
      else:
        range = float('inf')
        intersection = None
        return range, intersection
    
  def rayTracer(self, x, y, a):
    range = float('inf')
    intersection = None
    for obj in self.objects:
      # print "object.xCenter:", obj.xCenter
      # print "object.yCenter:", obj.yCenter
      # print "object.radius:", obj.radius
      r, inter = self.circleIntersection(x, y, a, obj)
      if r < range:
        range = r
        intersection = inter
    return range, intersection
    
  def computeScan(self, xx, yy, aa, tt):
    # xx numpy array size (1,N) for xRobot
    # yy numpy array size (1,N) for yRobot
    # aa numpy array size (1,N) for headingRobot
    # tt numpy array size (1,N) for time
    
    s = Scan()
    s.theta = arange( -2.*pi/3., 2.*pi/3. + 4.*pi/3./666., 4.*pi/3./666.)
    
    if len(s.theta) > len(tt):
      raise NameError("tt vector is too small (< 667) : len(tt)=" + str(len(tt)))
    if len(xx) != len(tt):
      raise NameError('xx and tt do not have same length')
    if len(yy) != len(tt):
      raise NameError('yy and tt do not have same length')
    if len(aa) != len(tt):
      raise NameError('aa and tt do not have same length')
    
    s.tbeg  = tt[0]
    s.range = zeros( (len(s.theta)) )
    s.tt    = zeros( (len(s.theta)) )
    
    for i in range(len(s.theta)):
      t = tt[i]
      x = xx[i]
      y = yy[i]
      a = aa[i]
      s.tend = t
      s.tt[i] = t
      r, inter = self.rayTracer(x, y, a + s.theta[i])
      if inter != None:
        s.range[i] = r + random.normalvariate(0.0, self.sigma)
    
    return s
    
    
    
    
