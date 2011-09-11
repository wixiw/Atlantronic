# coding=utf-8
import numpy as np
import math
import random
import logging

import PointCloud
import clusterize
import ransac
import BaseClasses
from BaseClasses import FoundObject
    
class ScanProcessor2:
  def __init__(self):
    self.beacons = []
    self.objects = []
    self.reset()

    self.medianFilterWidth = 3
    self.clusterParams = clusterize.ClusterizeParams()
    
    self.minRatio = 2.0
    self.minNbPointsForAngleComputing = 8
  
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
    
    vPC = clusterize.clusterize(pc, self.clusterParams)
    
    
    for v in vPC:
      print "************"
#      print v.points
      means, stddev, vectors = BaseClasses.pca(v.points[0:2,:])
#      print "means:", means
#      print "stddev:", stddev
#      print "vectors:"
#      print vectors
      fobj = BaseClasses.FoundObject() 
      fobj.x = means[0]
      fobj.y = means[1]
      fobj.nbPoints = v.points.shape[1]
      fobj.detectionTime = np.mean(v.points[2,:])
      
      index = np.array( range(tt.shape[0]) )[tt > fobj.detectionTime]
      if index.shape[0] == 0:
        raise ValueError("detectionTime is not in tt range : min(tt)=%f  max(tt)=%f and detectionTime=%f" % (np.min(tt), np.max(tt), fobj.detectionTime) )
      x_ = xx[index[0]]
      y_ = yy[index[0]]
      alpha = BaseClasses.betweenMinusPiAndPlusPi(math.atan2(fobj.y - y_, fobj.x - x_))
      fobj.detectionTheta = BaseClasses.betweenMinusPiAndPlusPi(alpha - hh[index[0]])
      fobj.detectionRange = math.sqrt( (fobj.y - y_)**2 + (fobj.x - x_)**2 )
      
      ratio = np.max(stddev) / np.min(stddev)
      print "x=", fobj.x
      print "y=", fobj.y
      print "ratio=", ratio
      print "nbPoints=", fobj.nbPoints
      if ratio >= self.minRatio and fobj.nbPoints >= self.minNbPointsForAngleComputing:
        sgmtDir = vectors[0:, np.argmax(stddev)]
        laserDir = np.array( [math.cos(alpha), math.sin(alpha)] )
        if laserDir[0] * sgmtDir[1] - laserDir[1] * sgmtDir[0] > 0.:
          sgmtDir = -sgmtDir
        beta = math.atan2( sgmtDir[1], sgmtDir[0] )
        fobj.h = BaseClasses.betweenMinusPiAndPlusPi( beta + np.pi/2.)
        print "h=", fobj.h * 180. / np.pi, "°"
        fobj.cap =  BaseClasses.betweenMinusPiAndPlusPi( beta - alpha + np.pi/2.)
        
      self.objects.append(fobj)
    
    
    
  
  def getBeacons(self, time, epsilon_time):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None
    heading = None
    if self.beacons == []:
      print "WARNING: ScanProcessor.getBeacons(): No beacon registred"
      return (xBeacon, yBeacon, range, theta, heading)
    
    for o in self.objects:
      if o.detectionTime < time + epsilon_time/2. and o.detectionTime >= time - epsilon_time/2. and o.used == False:
        o.used = True
        dist = np.array([ math.sqrt((b.xCenter-o.xCenter)**2 + (b.yCenter-o.yCenter)**2) for b in self.beacons ])
        
        minDist = np.min(dist)
        iMin = np.argmin(dist)
        if minDist < 0.7:
          xBeacon = o.x
          yBeacon = o.y
          range  = o.detectionRange
          theta  = o.detectionTheta
          heading = o.cap
        else:
          print "WARNING : the targeted beacon is too far !"
          
    return (xBeacon, yBeacon, range, theta, heading)

  def getTrueBeacons(self, index):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None    
    return (xBeacon, yBeacon, range, theta)
    

  