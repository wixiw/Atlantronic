from numpy import *
import math
import random

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
    
class ScanProcessor:
  def __init__(self):
    self.beacons = []
    self.objects = []
    self.objIndex = []
    self.scan = Scan()
    self.thresholdRange = 0.05
    
    self.currentIndex = -1
    self.currentPts = [[],[]]
    self.clusterOpen = False
    
  def setScan(self, s):
    self.scan = s
    
  def openCluster(self, r, t, x, y):
    self.currentIndex = self.currentIndex + 1
    self.currentPts = [[x+ r * cos(t)],
                       [y + r * sin(t)]]
    self.clusterOpen = True
    # print "open new cluster with theta:", t*180./pi, "and range:",r
    
  def closeCluster(self, r, t, x, y):
    obj = Object()
    xMean = mean(self.currentPts[0])
    yMean = mean(self.currentPts[1])
    obj.pts = self.currentPts
    xA = self.currentPts[0][0]
    yA = self.currentPts[1][0]
    xB = self.currentPts[0][len(self.currentPts[0])-1]
    yB = self.currentPts[1][len(self.currentPts[0])-1]
    obj.radius = math.sqrt( (xB-xA)**2 + (yB-yA)**2 ) / 2.
    obj.xCenter = mean(self.currentPts[0]) + 0.85 * obj.radius * cos(math.atan2(yMean-y, xMean-x))
    obj.yCenter = mean(self.currentPts[1]) + 0.85 * obj.radius * sin(math.atan2(yMean-y, xMean-x))
    # obj.xCenter = xMean
    # obj.yCenter = yMean
    self.objects.append( obj )
    self.currentPts = [[],[]]
    self.clusterOpen = False
    # print "close cluster with theta:", t*180./pi, "and range:",r
    
    
  def addToCurrentCluster(self, r, t, x, y):
      self.currentPts[0].append( x + r * cos(t) )
      self.currentPts[1].append( y + r * sin(t) )
    
    
    
  def findCluster(self, xx, yy, aa):
    theta = array( [ self.scan.theta[i] + aa[i] for i in range(len(self.scan.theta)) ] )
    prev = 0.
    for i in range(len(self.scan.range)):
      r = self.scan.range[i]
      t = theta[i]
      if r > 0.:
        if prev > 0.:
          if self.clusterOpen:
            if abs(r - prev) > self.thresholdRange:
              print "range threshold hook"
              self.closeCluster(r,t, xx[i], yy[i])
            else:
              self.addToCurrentCluster(r,t, xx[i], yy[i])              
          else:
            if abs(r - prev) < self.thresholdRange:
              self.openCluster(r, t, xx[i], yy[i])
        else:
          if i > 0:
            self.openCluster(r, t, xx[i], yy[i])
      else:
        if self.clusterOpen:
          self.closeCluster(r,t, xx[i], yy[i])
      if self.clusterOpen:
        self.objIndex.append(self.currentIndex)
      else:
        self.objIndex.append(None)
      prev = r
    
  def getBalise(self, t):
    theta = None
    range = None
    xBeacon = None
    yBeacon = None
    return theta, range, xBeacon, yBeacon
    
  def getNearestBeacons(self, obj):
    if self.beacons == []:
      return None
    dist = array([ (b.xCenter-obj.xCenter)**2 + (b.xCenter-obj.xCenter)**2 for b in self.beacons ])
    iMin = argmin(dist)
    return self.beacons[iMin]