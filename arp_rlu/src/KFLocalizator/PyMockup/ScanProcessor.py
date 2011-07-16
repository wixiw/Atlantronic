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
    self.t_beg   = None
    self.t_end   = None
    
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
    self.t_last_open = None
    
  def setScan(self, s):
    self.scan = s
    
  # for internal use only
  def openCluster(self, r, t, x, y):
    self.currentIndex = self.currentIndex + 1
    self.currentPts = [[x+ r * cos(t)],
                       [y + r * sin(t)]]
    self.clusterOpen = True
    self.t_last_open = t
    
  # for internal use only
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
    # le 0.85 est statistique. Il permet de reculer le point pour arriver pres du centre
    obj.xCenter = mean(self.currentPts[0]) + 0.85 * obj.radius * cos(math.atan2(yMean-y, xMean-x))
    obj.yCenter = mean(self.currentPts[1]) + 0.85 * obj.radius * sin(math.atan2(yMean-y, xMean-x))
    obj.t_beg = self.t_last_open
    obj.t_end = t
    self.objects.append( obj )
    self.currentPts = [[],[]]
    self.clusterOpen = False
    
  # for internal use only
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
    
    
  def getBeacons(self, t):
    xBeacon = None
    yBeacon = None
    if self.beacons == []:
      return xBeacon, yBeacon
    for o in self.objects:
      if t < (o.t_beg + o.t_end)/2. + 0.00005 and t >= (o.t_beg + o.t_end)/2. - 0.00005:
        dist = array([ (b.xCenter-obj.xCenter)**2 + (b.xCenter-obj.xCenter)**2 for b in self.beacons ])
        minDist = min(dist)
        iMin = argmin(dist)
        if minDist < 0.2:
          xBeacon = self.beacons[iMin].xCenter
          yBeacon = self.beacons[iMin].yCenter
    return xBeacon, yBeacon
    