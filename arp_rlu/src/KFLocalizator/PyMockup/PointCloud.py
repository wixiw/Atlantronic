import numpy as np
import math
from abc import ABCMeta
import logging

class PointCloud:
  def __init__(self, N = 0):
    self.tt = np.zeros( (N) )
    self.points = np.zeros( (2,N))
    
  def fromScan(self, scan, tt, xx, yy, hh):
    log = logging.getLogger('fromScan')
    n = scan.theta.shape[0]
    if type(tt) is float:
      tt = [tt] * n
    if type(xx) is float:
      xx = [xx] * n
    if type(yy) is float:
      yy = [yy] * n
    if type(hh) is float:
      hh = [hh] * n
    if len(tt) != n:
      raise TypeError("tt should be scalars or list with the same length than scan.theta. len(tt)=%d" % len(tt))
    if len(xx) != n:
      raise TypeError("xx should be scalars or list with the same length than scan.theta. len(xx)=%d" % len(xx))
    if len(yy) != n:
      raise TypeError("yy should be scalars or list with the same length than scan.theta. len(yy)=%d" % len(yy))
    if len(hh) != n:
      raise TypeError("hh should be scalars or list with the same length than scan.theta. len(hh)=%d" % len(hh))
    self.points = np.zeros((2, n ))
    self.points[0,0:] = xx[0:] + scan.range[0:] * np.cos(scan.theta[0:] + hh[0:])
    self.points[1,0:] = yy[0:] + scan.range[0:] * np.sin(scan.theta[0:] + hh[0:])
    self.tt = np.array(tt)
    
  def cleanUp(self):
    self.tt  = self.tt[np.nonzero(self.points[1,0:])]
    self.points = np.vstack( (self.points[0, np.nonzero(self.points[1,0:])], self.points[1, np.nonzero(self.points[1,0:])]) )
    