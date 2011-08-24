from numpy import *
import math
import random

from BaseClasses import *
    
class ScanProcessor:
  def __init__(self):
    self.beacons = []
    self.objects = []
    self.objIndex = []
    self.scan = Scan()
    self.thresholdRange = 0.05
    
    self.currentIndex = -1
    self.currentPts = [[],[],[],[]]
    self.clusterOpen = False
    self.thetaLastOpen = None
    self.thetaLastAdded = None
    self.timeLastOpen = None
    self.timeLastAdded = None
    
  def reset(self):
    self.objects = []
    self.objIndex = []
    self.currentIndex = -1
    self.currentPts = [[],[],[],[]]
    self.clusterOpen = False
    self.thetaLastOpen = None
    self.thetaLastAdded = None
    self.timeLastOpen = None
    self.timeLastAdded = None
    
    
  def setScan(self, s):
    self.reset()
    self.scan = s
    
  def setTrueStaticPositionForDebugOnly(self, x, y, h):
    self.trueX = x
    self.trueY = y
    self.trueH = h
    
  # for internal use only
  def openCluster(self, t, r, theta, x, y, h):
    self.currentIndex = self.currentIndex + 1
    self.currentPts = [[x + r * cos(theta + h)],
                       [y + r * sin(theta + h)],
                       [r],
                       [theta]]
    self.clusterOpen = True
    self.thetaLastOpen = theta
    self.timeLastOpen = t
    self.thetaLastAdded = theta
    self.timeLastAdded = t
    
  # for internal use only
  def closeCluster(self, t, r, theta, x, y, h):
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
    obj.thetaEnd = self.thetaLastAdded
    obj.timeBeg = self.timeLastOpen
    obj.timeEnd = self.timeLastAdded
    obj.range = mean(self.currentPts[2]) + 0.85 * obj.radius
    self.objects.append( obj )
    self.currentPts = [[],[],[],[]]
    self.clusterOpen = False
    
  # for internal use only
  def addToCurrentCluster(self, t, r, theta, x, y, h):
      self.currentPts[0].append( x + r * cos(theta + h) )
      self.currentPts[1].append( y + r * sin(theta + h) )
      self.currentPts[2].append( r )
      self.currentPts[3].append( theta )
      self.thetaLastAdded = theta
      self.timeLastAdded = t
      
    
    
  def findCluster(self, tt, xx, yy, hh):
    thetatheta = array( [ self.scan.theta[i] + hh[i] for i in range(len(self.scan.theta)) ] )
    prev = 0.
    for i in range(len(self.scan.range)):
      r = self.scan.range[i]
      t = tt[i]
      theta = self.scan.theta[i]
      if r > 0.:
        if prev > 0.:
          if self.clusterOpen:
            if abs(r - prev) > self.thresholdRange:
              self.closeCluster(t, r, theta, xx[i], yy[i], hh[i])
            else:
              self.addToCurrentCluster(t, r, theta, xx[i], yy[i], hh[i])              
          else:
            if abs(r - prev) < self.thresholdRange:
              self.openCluster(t, r, theta, xx[i], yy[i], hh[i])
        else:
          if i > 0:
            self.openCluster(t, r, theta, xx[i], yy[i], hh[i])
      else:
        if self.clusterOpen:
          self.closeCluster(t, r, theta, xx[i], yy[i], hh[i])
      if self.clusterOpen:
        self.objIndex.append(self.currentIndex)
      else:
        self.objIndex.append(None)
      prev = r
    
    
  def getBeacons(self, time):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None
    if self.beacons == []:
      # print "WARNING: ScanProcessor.getBeacons(): No beacon registred"
      return (xBeacon, yBeacon, range, theta)
    for o in self.objects:
      # we select the object detected at time t (if exist)
      # print "ScanProcessor.getBeacons(): o.thetaBeg=", o.thetaBeg, "  time=", time, "  o.thetaEnd=", o.thetaEnd
      if time > (o.timeBeg + o.timeEnd)/2. and o.used == False :
        o.used = True
        dist = array([ sqrt((b.xCenter-o.xCenter)**2 + (b.yCenter-o.yCenter)**2) for b in self.beacons ])
        #=======================================================================
        # print "-----------------------------------------"
        # print "ScanProcessor.getBeacons(): min time=", (o.timeBeg + o.timeEnd)/2. - 0.1/2048.
        # print "ScanProcessor.getBeacons(): time    =", time
        # print "ScanProcessor.getBeacons(): max time=", (o.timeBeg + o.timeEnd)/2. + 0.1/2048.
        # print "ScanProcessor.getBeacons(): timeBeg=", o.timeBeg
        # print "ScanProcessor.getBeacons(): timeEnd=", o.timeEnd
        # print "ScanProcessor.getBeacons(): o.xCenter=", o.xCenter
        # print "ScanProcessor.getBeacons(): o.yCenter=", o.yCenter
        # print "ScanProcessor.getBeacons(): beacons=", [ [b.xCenter, b.yCenter] for b in self.beacons ]
        # print "ScanProcessor.getBeacons(): dist=", dist
        #=======================================================================
        
        minDist = min(dist)
        iMin = argmin(dist)
        if minDist < 0.7:
          xBeacon = self.beacons[iMin].xCenter
          yBeacon = self.beacons[iMin].yCenter
          range  = o.range
          theta  = (o.thetaEnd + o.thetaBeg) / 2.
        else:
          print "WARNING : the targeted beacon is too far !"
    return (xBeacon, yBeacon, range, theta)

  def getTrueBeacons(self, index):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None
    minTheta = min(self.scan.theta)
    maxTheta = max(self.scan.theta)
    thetaStep = betweenMinusPiAndPlusPi((maxTheta - minTheta)/(len(self.scan.theta)-1))
    for b in self.beacons:
      thetaBeac = betweenMinusPiAndPlusPi(math.atan2(b.yCenter - self.trueY, b.xCenter - self.trueX) -  self.trueH)
      if self.scan.theta[index] > thetaBeac - thetaStep/2. and self.scan.theta[index] <= thetaBeac + thetaStep/2.:
        xBeacon = b.xCenter
        yBeacon = b.yCenter
        range = sqrt((self.trueX - xBeacon)**2 + (self.trueY - yBeacon)**2 )
        theta = thetaBeac
        return (xBeacon, yBeacon, range, theta)
    return (xBeacon, yBeacon, range, theta)

  def printObjects(self):
    for o in self.objects:
      print "---"
      print " xCenter=",o.xCenter
      print " yCenter=",o.yCenter
      print " radius=",o.radius
      print " thetaBeg=",o.thetaBeg
      print " thetaEnd",o.thetaEnd
      print " timeBeg=",o.timeBeg
      print " timeEnd=",o.timeEnd
    