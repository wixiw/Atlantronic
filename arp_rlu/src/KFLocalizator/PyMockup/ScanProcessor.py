from numpy import *
import math
import random

from BaseClasses import Scan
from BaseClasses import Object
    
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
    self.thetaLastOpen = None
    self.timeLastOpen = None
    
  def setScan(self, s):
    self.scan = s
    
  # for internal use only
  def openCluster(self, t, r, theta, x, y):
    self.currentIndex = self.currentIndex + 1
    self.currentPts = [[x + r * cos(theta)],
                       [y + r * sin(theta)]]
    self.clusterOpen = True
    self.thetaLastOpen = theta
    self.timeLastOpen = t
    
  # for internal use only
  def closeCluster(self, t, r, theta, x, y):
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
    obj.thetaBeg = self.thetaLastOpen
    obj.thetaEnd = theta
    obj.timeBeg = self.timeLastOpen
    obj.timeEnd = t
    self.objects.append( obj )
    self.currentPts = [[],[]]
    self.clusterOpen = False
    
  # for internal use only
  def addToCurrentCluster(self, t, r, theta, x, y):
      self.currentPts[0].append( x + r * cos(theta) )
      self.currentPts[1].append( y + r * sin(theta) )
    
    
  def findCluster(self, tt, xx, yy, hh):
    thetatheta = array( [ self.scan.theta[i] + hh[i] for i in range(len(self.scan.theta)) ] )
    prev = 0.
    for i in range(len(self.scan.range)):
      r = self.scan.range[i]
      t = tt[i]
      theta = thetatheta[i]
      if r > 0.:
        if prev > 0.:
          if self.clusterOpen:
            if abs(r - prev) > self.thresholdRange:
              print "range threshold hook"
              self.closeCluster(t, r, theta, xx[i], yy[i])
            else:
              self.addToCurrentCluster(t, r, theta, xx[i], yy[i])              
          else:
            if abs(r - prev) < self.thresholdRange:
              self.openCluster(t, r, theta, xx[i], yy[i])
        else:
          if i > 0:
            self.openCluster(t, r, theta, xx[i], yy[i])
      else:
        if self.clusterOpen:
          self.closeCluster(t, r, theta, xx[i], yy[i])
      if self.clusterOpen:
        self.objIndex.append(self.currentIndex)
      else:
        self.objIndex.append(None)
      prev = r
    
    
  def getBeacons(self, time):
    xBeacon = None
    yBeacon = None
    radius = None
    theta = None
    if self.beacons == []:
      return (xBeacon, yBeacon)
    for o in self.objects:
      # we select the object detected at time t (if exist)
      if time < (o.timeBeg + o.timeEnd)/2. + 0.00005 and time >= (o.timeBeg + o.timeEnd)/2. - 0.00005:
        dist = array([ (b.xCenter-o.xCenter)**2 + (b.xCenter-o.xCenter)**2 for b in self.beacons ])
        minDist = min(dist)
        iMin = argmin(dist)
        if minDist < 0.2:
          xBeacon = self.beacons[iMin].xCenter
          yBeacon = self.beacons[iMin].yCenter
          radius  = o.radius
          theta  = (o.thetaEnd + o.thetaBegin) / 2
    return (xBeacon, yBeacon, radius, theta)
    