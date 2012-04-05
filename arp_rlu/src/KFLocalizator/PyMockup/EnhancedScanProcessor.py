# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy

import numpy as np
import math
import random


import BaseMethods
from BaseClasses import *
    
class EnhancedScanProcessor:
  def __init__(self):
    self.trueBeacons = []
    self.trueBeacons.append(Circle(x =  1.5, y =  0.0, r = 0.04))
    self.trueBeacons.append(Circle(x = -1.5, y =  1.0, r = 0.04))
    self.trueBeacons.append(Circle(x = -1.5, y = -1.0, r = 0.04))
    self.refSmallLength = 2.0
    self.refBigLength = math.sqrt(10)
    
    self.thresholdRange = 0.1
    self.maxRadius = 0.1
    
    self.scan = Scan()
    self.reset()
    
    self.oneObjectParams = {
      "maxDistance" : 0.1, 
    }
    
    self.twoObjectsParams = {
      "maxDistance" : 0.3,
      "maxSmallSegmentVariation" : 0.1,
      "maxBigSegmentVariation" : 0.1,
    }
    
    self.treeObjectsParams = {
      "maxDistance" : 0.3,
      "maxTriangleVariation" : 0.05,
    }
    
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
    
    self.foundBeacons = []
    
    
  def setScan(self, s):
    self.reset()
    self.scan = s
    
  def setTrueStaticPositionForDebugOnly(self, x, y, h):
    self.trueX = x
    self.trueY = y
    self.trueH = h
    
  def __openCluster(self, t, r, theta, x, y, h):
    self.currentIndex = self.currentIndex + 1
    self.currentPts = [[x + r * math.cos(theta + h)],
                       [y + r * math.sin(theta + h)],
                       [r],
                       [theta]]
    self.clusterOpen = True
    self.thetaLastOpen = theta
    self.timeLastOpen = t
    self.thetaLastAdded = theta
    self.timeLastAdded = t
    
  def __closeCluster(self, t, r, theta, x, y, h):
    obj = Circle()
    xMean = np.mean(self.currentPts[0])
    yMean = np.mean(self.currentPts[1])
    obj.pts = self.currentPts
    xA = self.currentPts[0][0]
    yA = self.currentPts[1][0]
    xB = self.currentPts[0][len(self.currentPts[0])-1]
    yB = self.currentPts[1][len(self.currentPts[0])-1]
#    obj.radius = math.sqrt( (xB-xA)**2 + (yB-yA)**2 ) / 2.
    obj.radius = 0.04
    # le 0.85 est statistique. Il permet de reculer le point pour arriver pres du centre
    obj.xCenter = np.mean(self.currentPts[0]) + 0.85 * obj.radius * math.cos(math.atan2(yMean-y, xMean-x))
    obj.yCenter = np.mean(self.currentPts[1]) + 0.85 * obj.radius * math.sin(math.atan2(yMean-y, xMean-x))
    obj.thetaBeg = self.thetaLastOpen
    obj.thetaEnd = self.thetaLastAdded
    obj.timeBeg = self.timeLastOpen
    obj.timeEnd = self.timeLastAdded
    obj.range = np.mean(self.currentPts[2]) + 0.85 * obj.radius
#    rospy.loginfo( "EnhancedScanProcessor - xMean=%f", xMean )
#    rospy.loginfo( "EnhancedScanProcessor - yMean=%f", yMean )
#    rospy.loginfo( "EnhancedScanProcessor - x=%f", x )
#    rospy.loginfo( "EnhancedScanProcessor - y=%f", y )
#    rospy.loginfo( "EnhancedScanProcessor - xMean - x=%f", xMean - x )
#    rospy.loginfo( "EnhancedScanProcessor - yMean - y=%f", yMean - y )
#    rospy.loginfo( "EnhancedScanProcessor - math.atan2(yMean-y, xMean-x)=%f", math.atan2(yMean-y, xMean-x) )
#    rospy.loginfo( "EnhancedScanProcessor - xDelta=%f", 0.85 * obj.radius * math.cos(math.atan2(yMean-y, xMean-x)) )
#    rospy.loginfo( "EnhancedScanProcessor - yDelta=%f", 0.85 * obj.radius * math.sin(math.atan2(yMean-y, xMean-x)) )
#    rospy.loginfo( "EnhancedScanProcessor - xCenter=%f", obj.xCenter )
#    rospy.loginfo( "EnhancedScanProcessor - yCenter=%f", obj.yCenter )
#    rospy.loginfo( "EnhancedScanProcessor - ranges=%s", repr(self.currentPts[2]) )
#    rospy.loginfo( "EnhancedScanProcessor - mean(range)=%f", np.mean(self.currentPts[2]) )
#    rospy.loginfo( "EnhancedScanProcessor - range=%f", obj.range )
#    rospy.loginfo( "EnhancedScanProcessor - theta=%f", (obj.thetaBeg + obj.thetaEnd)/2. )
#    rospy.loginfo( "EnhancedScanProcessor - time=%f", (obj.timeBeg + obj.timeEnd)/2. )
#    rospy.loginfo( "EnhancedScanProcessor - ----------------" )
    self.objects.append( obj )
    self.currentPts = [[],[],[],[]]
    self.clusterOpen = False
    
  def __addToCurrentCluster(self, t, r, theta, x, y, h):
      self.currentPts[0].append( x + r * math.cos(theta + h) )
      self.currentPts[1].append( y + r * math.sin(theta + h) )
      self.currentPts[2].append( r )
      self.currentPts[3].append( theta )
      self.thetaLastAdded = theta
      self.timeLastAdded = t
      
  def do(self, scan, tt, xx, yy, hh):
    self.setScan(scan)
    self.findClusters(tt, xx, yy, hh)
    self.filterClusters()
    self.associateClusters()
    self.cleanResults()
    
    
  def findClusters(self, tt, xx, yy, hh):
    thetatheta = np.array( [ self.scan.theta[i] + hh[i] for i in range(len(self.scan.theta)) ] )
    prev = 0.
    for i in range(len(self.scan.range)):
      r = self.scan.range[i]
      t = tt[i]
      theta = self.scan.theta[i]
      if r > 0.:
        if prev > 0.:
          if self.clusterOpen:
            if abs(r - prev) > self.thresholdRange:
              self.__closeCluster(t, r, theta, xx[i], yy[i], hh[i])
            else:
              self.__addToCurrentCluster(t, r, theta, xx[i], yy[i], hh[i])              
          else:
            if abs(r - prev) < self.thresholdRange:
              self.__openCluster(t, r, theta, xx[i], yy[i], hh[i])
        else:
          if i > 0:
            self.__openCluster(t, r, theta, xx[i], yy[i], hh[i])
      else:
        if self.clusterOpen:
          self.__closeCluster(t, r, theta, xx[i], yy[i], hh[i])
      if self.clusterOpen:
        self.objIndex.append(self.currentIndex)
      else:
        self.objIndex.append(None)
      prev = r
    
      
  def filterClusters(self):
    n = len(self.objects)
    for i in range(1,n+1):
      if self.objects[n-i].radius > self.maxRadius:
        self.objects.pop(n-i)
        
        
  def associateClusters(self):
    n = len(self.objects)
    self.foundBeacons = []
    if n == 0:
      rospy.loginfo("0 candidates => 0 beacons found")
    elif n == 1:
      rospy.loginfo("1 candidate")
      self.foundBeacons = self.__recognizeOneObject(self.oneObjectParams, self.objects[0])
    elif n == 2:
      rospy.loginfo("2 candidates")
      fb = self.recognizeTwoObjects(self.twoObjectsParams, self.objects[0], self.objects[1])
      if len(fb) == 0:
#        rospy.loginfo("fail to find segment => switch to recognizeOneObject")
        fb1 = self.__recognizeOneObject(self.oneObjectParams, self.objects[0])
        fb2 = self.__recognizeOneObject(self.oneObjectParams, self.objects[1])
        self.foundBeacons.extend(fb1)
        self.foundBeacons.extend(fb2)
      else:
        self.foundBeacons = fb
    else: # n >= 3:
      rospy.loginfo("3 or more candidates")
      fb = self.recognizeTreeOrMoreObjects(self.treeObjectsParams, self.objects)
      if len(fb) == 0:
#        rospy.loginfo("fail to find triangle or segment => switch to recognizeOneObject")
        for o in self.objects:
          fb = self.__recognizeOneObject(self.oneObjectParams, o)
          alreadyInFb = False
          for v in self.foundBeacons:
            for f in fb:
              if v.xCenter == f.xCenter and v.yCenter == f.yCenter:
                alreadyInFb = True
                break
          if not alreadyInFb:
            self.foundBeacons.extend(fb)
      else:
        self.foundBeacons = fb
        
  def cleanResults(self, fb = None):
#    rospy.loginfo("********* Cleaning *********")
    if fb is None:
      fb = self.foundBeacons
    for b in self.trueBeacons:
#      rospy.loginfo('beacon (%f,%f)', b.xCenter, b.yCenter)
      dist = []
      nb = 0
      for i,f in enumerate(fb):
        d = math.sqrt((b.xCenter - f.xCenter)**2 + (b.yCenter - f.yCenter)**2)
#        rospy.loginfo('  object[%d] (%f,%f)  d:%f',i , f.xCenter, f.yCenter, d)
        if d < 0.5:
          dist.append(d)
          nb= nb+1
        else:
          dist.append(float('+inf'))
#      rospy.loginfo("=> dist=%s", str(dist))
      if nb > 1:
#        rospy.loginfo("nb > 1")
#        rospy.loginfo("argmin(dist):%d",np.argmin(dist))
        for i in range(len(fb)-1, -1, -1):
#          rospy.loginfo("i:%d  d:%f",i, dist[i])
          if dist[i] < 0.5 and i != np.argmin(dist):
#            rospy.loginfo("pop(%d)",i)
            fb.pop(i)
    self.foundBeacons = fb
    return fb

  def __recognizeOneObject(self, params, o):
    fb = []
    minDist = float('+inf')
    bestBeacon = None
    for b in self.trueBeacons:
      d = math.sqrt( (o.xCenter - b.xCenter )**2 + ( o.yCenter - b.yCenter)**2 )
      if d < minDist:
        bestBeacon = b
        minDist = d
    if minDist < params["maxDistance"]:
      fb.append(o)
#      rospy.loginfo("one object (%f,%f) near beacon (%f,%f) with distance %f (against %f)", o.xCenter, o.yCenter, bestBeacon.xCenter, bestBeacon.yCenter, minDist, params["maxDistance"])
#    else:
#      rospy.loginfo("one object (%f,%f) is too far from nearest beacon (%f,%f) with distance %f (against %f)=> 0 beacons found", o.xCenter, o.yCenter, bestBeacon.xCenter, bestBeacon.yCenter, minDist, params["maxDistance"])
    return fb
  
  
  def recognizeTwoObjects(self, params, o1, o2):
    fb = []
    L = math.sqrt( (o1.xCenter - o2.xCenter)**2 + (o1.yCenter - o2.yCenter)**2 )
    if math.fabs(L - self.refSmallLength) < params["maxSmallSegmentVariation"]:
#      rospy.loginfo("small segment has been found")
      fb1 = self.__recognizeOneObject(params, o1)
      fb2 = self.__recognizeOneObject(params, o2)
      if len(fb1) == 1 and len(fb2) == 1:
        fb.append(fb1[0])
        fb.append(fb2[0])
        rospy.loginfo("small segment has been found")
        return fb
      else:
#        rospy.loginfo("at least one beacon is too far from is target beacon")
        return []
    elif math.fabs(L - self.refBigLength) < params["maxBigSegmentVariation"]:
#      rospy.loginfo("big segment has been found")
      fb1 = self.__recognizeOneObject(params, o1)
      fb2 = self.__recognizeOneObject(params, o2)
      if len(fb1) == 1 and len(fb2) == 1:
        fb.append(fb1[0])
        fb.append(fb2[0])
        rospy.loginfo("big segment has been found")
        return fb
      else:
#        rospy.loginfo("at least one beacon is too far from is target beacon")
        return []
    else:
#      rospy.loginfo("no segment recognized : (%f,%f) & (%f,%f) width L=%f against %f or %f", o1.xCenter, o1.yCenter, o2.xCenter, o2.yCenter, L, self.refSmallLength, self.refBigLength)
      return []
  
  
  def recognizeTreeOrMoreObjects(self, params, oo):
    fb = []
    bestComb = None
    bestQuality = 0.
    combs = BaseMethods.enumerateCombinations(range(len(oo)),3)
    for comb in combs:
      quality = self.__evalCombination(oo, comb)
#      rospy.loginfo("eval comb %s => quality is %f", str(comb), quality)
      if quality > bestQuality:
#        rospy.loginfo("best quality !")
        bestComb = comb
        bestQuality = quality
    if bestQuality < 1. / params["maxTriangleVariation"]:
      rospy.loginfo("Unable to recognize triangle in candidates [%d,%d,%d] : variation incorrect (%f against %f) => we try recognizeTwoObjects",bestComb[0],bestComb[1],bestComb[2], 1. / bestQuality, params["maxTriangleVariation"])
      combs = BaseMethods.enumerateCombinations(range(len(oo)),2)
#      rospy.loginfo("combinations to be tryed:")
#      for c in combs:
#        rospy.loginfo("  (%d,%d)", c[0], c[1])
      for comb in combs:
#        rospy.loginfo("considering (%d,%d)", comb[0], comb[1])
        vv = self.recognizeTwoObjects(self.twoObjectsParams, oo[comb[0]], oo[comb[1]])
        alreadyInFb = False
        for v in vv:
          for f in fb:
            if v.xCenter == f.xCenter and v.yCenter == f.yCenter:
              alreadyInFb = True
              break
          if not alreadyInFb:
            fb.append(v)
      return fb
    else:
      rospy.loginfo("Triangle found in candidates [%d,%d,%d] (variation: %f  against %f)",bestComb[0],bestComb[1],bestComb[2], 1. / bestQuality, params["maxTriangleVariation"])
      fb.append(oo[bestComb[0]])
      fb.append(oo[bestComb[1]])
      fb.append(oo[bestComb[2]])
    return fb
    
    
  def __evalCombination(self, oo, comb):
    A = np.array( [[oo[comb[0]].xCenter],[oo[comb[0]].yCenter]] )
    B = np.array( [[oo[comb[1]].xCenter],[oo[comb[1]].yCenter]] )
    C = np.array( [[oo[comb[2]].xCenter],[oo[comb[2]].yCenter]] )
    ll = []
    ll.append( np.linalg.norm(B - A) )
    ll.append( np.linalg.norm(C - A) )
    ll.append( np.linalg.norm(C - B) )
    maxll = np.max(ll)
    medll = BaseMethods.getMedian(ll)
    minll = np.min(ll)
    maxDiff = math.fabs(maxll - self.refBigLength)
    medDiff = math.fabs(medll - self.refBigLength)
    minDiff = math.fabs(minll - self.refSmallLength)
    invquality =  np.max( [maxDiff, medDiff, minDiff] )
    return 1./ invquality
    
    
  def getBeacons(self, time):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None
    if self.trueBeacons == []:
      rospy.logwarn("No beacon registred")
      return (xBeacon, yBeacon, range, theta)
    if len(self.foundBeacons) < 2:
      return (xBeacon, yBeacon, range, theta)
    for o in self.foundBeacons:
      if time > (o.timeBeg + o.timeEnd)/2. and o.used == False :
        o.used = True
        dist = np.array([ math.sqrt((b.xCenter-o.xCenter)**2 + (b.yCenter-o.yCenter)**2) for b in self.trueBeacons ])        
        minDist = np.min(dist)
        iMin = np.argmin(dist)
        if minDist < 0.5:
          xBeacon = self.trueBeacons[iMin].xCenter
          yBeacon = self.trueBeacons[iMin].yCenter
          range  = o.range
          theta  = (o.thetaEnd + o.thetaBeg) / 2.
#          rospy.loginfo("==== Time %f", time)
#          rospy.loginfo("dist=%s", str(dist))
#          rospy.loginfo("iMin=%d with minDist=%f", iMin, minDist)
#          rospy.loginfo("xBeacon=%f  yBeacon=%f",xBeacon, yBeacon)
#          rospy.loginfo("xCenter=%f  yCenter=%f",o.xCenter, o.yCenter)
#          rospy.loginfo("range=%f  theta=%f", range, theta)
        else:
          rospy.logwarn("the targeted beacon is too far !")
    return (xBeacon, yBeacon, range, theta)

  def getTrueBeacons(self, index):
    xBeacon = None
    yBeacon = None
    range = None
    theta = None
    minTheta = np.min(self.scan.theta)
    maxTheta = np.max(self.scan.theta)
    thetaStep = betweenMinusPiAndPlusPi((maxTheta - minTheta)/(len(self.scan.theta)-1))
    for b in self.trueBeacons:
      thetaBeac = betweenMinusPiAndPlusPi(math.atan2(b.yCenter - self.trueY, b.xCenter - self.trueX) -  self.trueH)
      if self.scan.theta[index] > thetaBeac - thetaStep/2. and self.scan.theta[index] <= thetaBeac + thetaStep/2.:
        xBeacon = b.xCenter
        yBeacon = b.yCenter
        range = math.sqrt((self.trueX - xBeacon)**2 + (self.trueY - yBeacon)**2 )
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
    
  
    