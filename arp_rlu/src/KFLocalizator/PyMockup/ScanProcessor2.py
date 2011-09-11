# coding=utf-8
import numpy as np
import math
import random
import logging

import PointCloud
import clusterize
import ransac
from BaseClasses import pca
from BaseClasses import FoundObject
    
class ScanProcessor2:
  def __init__(self):
    self.beacons = []
    self.objects = []
    self.reset()

    self.medianFilterWidth = 3
    self.clusterParams = clusterize.ClusterizeParams()
  
  def reset(self):
    self.objects = []
    
  def setTrueStaticPositionForDebugOnly(self, x, y, h):
    self.trueX = x
    self.trueY = y
    self.trueH = h
  
  def process(self, raw, tt, xx, yy, hh):
    self.reset()
    
    scan = raw.copy()
    scan.doMedianFiltering(self.medianFilterWidth)
    
    pc = PointCloud.PointCloud()
    pc.fromScan(scan, tt, xx, yy, hh)
    pc.cleanUp()
    
    vPC = clusterize.clusterize(pc, self.clusterParams)
    
    
    for v in vPC:
      print "************"
      print v.points
      means, stddev, vectors = pca(v.points[0:2,:])
      fobj = FoundObject() 
      fobj.x = means[0]
      fobj.y = means[1]
      fobj.nbPoints = v.points.shape[1]
      fobj.detectionTime = np.squeeze(np.mean(v.points[2,:]))
      ratio = np.max(stddev) / np.min(stddev)
      
      self.objects.append(fobj)
#      if ratio > 2.0:
#        sgmtDir = vectors[0:, np.argmax(stddev)]
#        if 
#        jobj.h = 
#      print "ratio:", np.max(stddev) / np.min(stddev)
#      print "sgmt direction:"
#      print vectors[0:, np.argmax(stddev)]
    
    
    
    
  
  def getBeacons(self, time):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None
    return (xBeacon, yBeacon, range, theta)

  def getTrueBeacons(self, index):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None    
    return (xBeacon, yBeacon, range, theta)
    

  