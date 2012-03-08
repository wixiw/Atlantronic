import numpy as np
import math
from abc import ABCMeta
import logging
import BaseMethods

import json

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
    
  def export(self, filename):
    dict = {}
    dict["type"] = "Scan"
    dict["size"] = len(self.tt)
    dict["tt"] = list(self.tt) #reduce(lambda  x,y: x + " " + str(y),  list(self.tt)[1:], str(list(self.tt)[0]))
    dict["range"] = list(self.range) #reduce(lambda  x,y: x + " " + str(y),  list(self.range)[1:], str(list(self.range)[0]))
    dict["theta"] = list(self.theta) #reduce(lambda  x,y: x + " " + str(y),  list(self.theta)[1:], str(list(self.theta)[0]))
    
    output = open(filename, mode='w')
    output.write(json.dumps(dict,indent=2,sort_keys=True))
    output.close()
    
  def load(self, filename):
    input = open(filename,mode='r')
    dict = json.loads(input.read())
    input.close()
    
    assert("type" in dict)
    assert(dict["type"] == "Scan")
    assert("size" in dict and "tt" in dict and "range" in dict and "theta" in dict)
    N = dict["size"]
    self.tsync = 0.
    self.tbeg  = 0.
    self.tend  = 0.
    self.tt = np.array(dict["tt"]) #np.array([ float(x) for x in dict["tt"].split()] )
    self.range = np.array(dict["range"]) #np.array([ float(x) for x in dict["range" ].split()] )
    self.theta = np.array(dict["theta"]) #np.array([ float(x) for x in dict["theta" ].split()] )
    self.xx = None
    self.yy = None
    self.hh = None
    if "xx" in dict:
      self.xx = np.array(dict["xx"])
    if "yy" in dict:
      self.yy = np.array(dict["yy"])
    if "hh" in dict:
      self.hh = np.array(dict["hh"])
      
    
  