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
    s.theta = arange( -2.*pi*340./1024., -2.*pi*340./1024. + 2.*pi*681./1024., 2.*pi/1024.)
    print "min s.theta (en deg):", min(s.theta)*180./pi
    print "max s.theta (en deg):", max(s.theta)*180./pi
    print "delta s.theta (en deg):", (max(s.theta) - min(s.theta))*180./pi
    print "len(s.theta):", len(s.theta)
    
    if len(s.theta) > len(tt):
      raise NameError("tt vector is too small (<", len(s.theta), ") : len(tt)=" + str(len(tt)))
    if len(xx) != len(tt):
      raise NameError('xx and tt do not have same length')
    if len(yy) != len(tt):
      raise NameError('yy and tt do not have same length')
    if len(aa) != len(tt):
      raise NameError('aa and tt do not have same length')
    
    ideb = len(tt) - len(s.theta)
    s.tbeg  = tt[ideb]
    s.range = zeros( (len(s.theta)) )
    s.tt    = zeros( (len(s.theta)) )
    s.tsync = tt[ideb] - 44. * 0.1 / 1024.
    
    for i in range(len(s.theta)):
      t = tt[ideb+i]
      x = xx[ideb+i]
      y = yy[ideb+i]
      a = aa[ideb+i]
      s.tend = t
      s.tt[i] = t
      r, inter = self.rayTracer(x, y, a + s.theta[i])
      if inter != None:
        s.range[i] = r + random.normalvariate(0.0, self.sigma)
    
    return s
    
    
    
    
