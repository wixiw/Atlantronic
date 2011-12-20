import numpy as np
import math
from abc import ABCMeta
import logging
import BaseMethods

class MedianFilter:
  def __init__(self, W):
    self.W = W
  def compute(self, raw):
    filt = np.copy(raw)
    infIndex = (self.W -1)/2
    supIndex = self.W -1 - (self.W -1)/2
    for j in range(int(raw.shape[0])):
      if j-infIndex < 0:
        continue
      if j+supIndex > int(raw.shape[0]) - 1:
        continue
      v = list(raw[(j-infIndex):(j+supIndex+1)])
      filt[j] = BaseMethods.getMedian(v)
    return filt

class Scan:
  def __init__(self, N=0):
    self.tsync = 0.
    self.tbeg  = 0.
    self.tend  = 0.
    self.tt    = np.zeros( (N) )
    self.theta = np.zeros( (N) )
    self.range = np.zeros( (N) )
    
  def copy(self):
    retval = Scan()
    retval.tsync = self.tsync
    retval.tbeg = self.tbeg
    retval.tend = self.tend
    retval.tt = np.copy(self.tt)
    retval.theta = np.copy(self.theta)
    retval.range = np.copy(self.range)
    return retval
  
  def check(self):
    log = logging.getLogger('Scan - check()')
    res = True
    if self.tt.shape != self.theta.shape:
      res = False
      log.warning("tt and theta do not have same shape")
    if self.tt.shape != self.range.shape:
      res = False
      log.warning("tt and range do not have same shape")
    return res
  
  def cleanUp(self):
    self.check()
    self.tt = self.tt[np.nonzero(self.range)]
    self.theta = self.theta[np.nonzero(self.range)]
    self.range = self.range[np.nonzero(self.range)]
    self.check()
  
  def doMedianFiltering(self, width):
    mf = MedianFilter(width)
    self.range = mf.compute(self.range)
  