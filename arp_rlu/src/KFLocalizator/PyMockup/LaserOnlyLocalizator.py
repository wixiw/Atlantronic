# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy

import numpy as np
import math
import random

import PointCloud
import clusterize
import ransac
import BaseClasses
import BaseMethods

import EnhancedScanProcessor

class LaserOnlyLocalizator:
  def __init__(self):
    self.reset()
    
    self.scanproc = EnhancedScanProcessor.EnhancedScanProcessor()
    
#    self.refSmallLength = 2.0
#    self.refBigLength = math.sqrt(10)
#    self.qualityThreshold = 1. / 0.20
    
    self.trueA = np.array([ [1.5], [0.] ])
    self.trueB = np.array([ [-1.5], [1.] ])
    self.trueC = np.array([ [-1.5], [-1.] ])
    
  def reset(self):
    self.estimate = BaseClasses.Estimate()
  
  def process(self, scan_):
      
    self.reset()
    
    scan = scan_.copy()
    
    tt = np.zeros_like(scan.range)
    xx = np.zeros_like(scan.range)
    yy = np.zeros_like(scan.range)
    hh = np.zeros_like(scan.range)
    
    self.scanproc.setScan(scan)
    self.scanproc.findClusters(tt, xx, yy, hh)
    self.scanproc.filterClusters()
    candidates = self.scanproc.objects
    
    if len(candidates) < 3:
      rospy.logwarn("Less than 3 candidates found")
      return False
    rospy.loginfo("Objects detected : %d", len(candidates))
    
    foundBeacons = self.scanproc.recognizeTreeOrMoreObjects(self.scanproc.treeObjectsParams, candidates)
    foundBeacons = self.scanproc.cleanResults(foundBeacons)
    if len(foundBeacons) < 3:
      rospy.logwarn("Triangle not found in candidates")
      return False
    
    p = self.__identifyTriangle(foundBeacons)
    if p == []:
      rospy.logwarn("Unable to identify triangle in found beacons")
      return False
    A = self.__findCenter(foundBeacons[p[0]])
    B = self.__findCenter(foundBeacons[p[1]])
    C = self.__findCenter(foundBeacons[p[2]])
    rospy.loginfo("First beacon is %s", str(A.transpose()))
    rospy.loginfo("Second beacon is %s", str(B.transpose()))
    rospy.loginfo("Third beacon is %s", str(C.transpose()))
    
    self.estimate = self.__computeEstimateFromTriangle(A, B, C)
    rospy.loginfo("LaserOnlyLocalizator estimate is : x=%f  y=%f  h=%f", self.estimate.xRobot, self.estimate.yRobot, self.estimate.hRobot)
    return True
  
  def __findTriangle(self, candidates):
    # find beacons in candidates
    rospy.loginfo("Try to find triangle in candidates...")
    bestComb = None
    bestQuality = None
    combs = BaseMethods.enumerateCombinations(range(len(candidates)),3)
    for comb in combs:
      quality = self.__evalCombination(candidates, comb)
      if bestComb is None:
        bestComb = comb
        bestQuality = quality
      else:
        if quality > bestQuality:
          bestComb = comb
          bestQuality = quality
    
    rospy.loginfo("Best combination is %s with quality=%f", str(bestComb), bestQuality)
          
    if bestQuality < self.qualityThreshold:
      rospy.logwarn("Unable to recognize triangle in clusters : quality incorrect")
      bestComb = None
      bestQuality = None
      
    return bestComb, bestQuality
    
    
  def __identifyTriangle(self, candidates):
    perms = BaseMethods.enumeratePermutations(range(3))
#    rospy.loginfo("Eval all permutations \n%s", str(perms))
    for p in perms:
#      rospy.loginfo("p = %s", str(p))
      A = self.__findCenter(candidates[p[0]])
      B = self.__findCenter(candidates[p[1]])
      C = self.__findCenter(candidates[p[2]])
#      rospy.loginfo("A =\n%s", str(A))
#      rospy.loginfo("B =\n%s", str(B))
#      rospy.loginfo("C =\n%s", str(C))
      bary = (A + B + C) / 3.
#      rospy.loginfo("bary =\n%s", str(bary))
      vA = np.vstack( (A - bary, np.array([[0]])) ).reshape((3,))
      vB = np.vstack( (B - bary, np.array([[0]])) ).reshape((3,))
      vC = np.vstack( (C - bary, np.array([[0]])) ).reshape((3,))
#      rospy.loginfo("vA =\n%s", str(vA))
#      rospy.loginfo("vB =\n%s", str(vB))
#      rospy.loginfo("vC =\n%s", str(vC))
      # check order
      if np.cross(vA, vB)[2] < 0:
#        rospy.loginfo("np.cross(vA, vB)[2] < 0")
        continue
      if np.cross(vB, vC)[2] < 0:
#        rospy.loginfo("np.cross(vB, vC)[2] < 0")
        continue
      if np.cross(vC, vA)[2] < 0:
#        rospy.loginfo("np.cross(vC, vA)[2] < 0")
        continue
      # check if first beacon is first PC
      if np.linalg.norm(C - B) > (self.scanproc.refBigLength + self.scanproc.refSmallLength) / 2.:
        continue
      return p
    return []
  
  
  def __computeEstimateFromTriangle(self, A,B,C):
    bary = (A + B + C) / 3.
    trueBary = ( self.trueA + self.trueB + self.trueC ) / 3.
    nC = np.cross( np.vstack( (B - A, np.array([[0]]))).reshape(3,) , np.array([0.,0.,1.]) ) 
    nA = np.cross( np.vstack( (C - B, np.array([[0]]))).reshape(3,) , np.array([0.,0.,1.]) ) 
    nB = np.cross( np.vstack( (A - C, np.array([[0]]))).reshape(3,) , np.array([0.,0.,1.]) )  
    nA = nA / np.linalg.norm(nA)
    nB = nB / np.linalg.norm(nB)
    nC = nC / np.linalg.norm(nC)
    trueNC = np.cross( np.vstack( (self.trueB - self.trueA, np.array([[0]]))).reshape(3,) , np.array([0.,0.,1.]) ) 
    trueNA = np.cross( np.vstack( (self.trueC - self.trueB, np.array([[0]]))).reshape(3,) , np.array([0.,0.,1.]) )
    trueNB = np.cross( np.vstack( (self.trueA - self.trueC, np.array([[0]]))).reshape(3,) , np.array([0.,0.,1.]) )
    trueNA = trueNA / np.linalg.norm(trueNA)
    trueNB = trueNB / np.linalg.norm(trueNB)
    trueNC = trueNC / np.linalg.norm(trueNC)
    
    angleA = math.atan2( np.cross(nA, trueNA)[2] , np.vdot(nA, trueNA))
    angleB = math.atan2( np.cross(nB, trueNB)[2] , np.vdot(nB, trueNB))
    angleC = math.atan2( np.cross(nC, trueNC)[2] , np.vdot(nC, trueNC))
    h = (angleA + angleB + angleC) / 3.
    
    x = trueBary[0,0] - bary[0,0] * math.cos(h) + bary[1,0] * math.sin(h) 
    y = trueBary[1,0] - bary[0,0] * math.sin(h) - bary[1,0] * math.cos(h)
    
    estim = BaseClasses.Estimate()
    estim.xRobot = x
    estim.yRobot = y
    estim.hRobot = h
    estim.covariance = np.diag((0.05, 0.05, 0.01))
    # todo : compute better covariance
    
    return estim
          
        
  def __evalCombination(self, candidates, comb):
    A = self.__findCenter(candidates[comb[0]])
    B = self.__findCenter(candidates[comb[1]])
    C = self.__findCenter(candidates[comb[2]])
    ll = []
    ll.append( np.linalg.norm(B - A) )
    ll.append( np.linalg.norm(C - A) )
    ll.append( np.linalg.norm(C - B) )
    maxll = np.max(ll)
    medll = BaseMethods.getMedian(ll)
    minll = np.min(ll)
#    rospy.loginfo("max lenght = %f",maxll)
#    rospy.loginfo("med lenght = %f",medll)
#    rospy.loginfo("min lenght = %f",minll)
    invquality = (maxll - self.refBigLength)**2 + (medll - self.refBigLength)**2 + (minll - self.refSmallLength)**2 
    return 1./ invquality
      
      
  def __findCenter(self, candidate):
    return np.array([[candidate.xCenter],[candidate.yCenter]])
  
  
  def getEstimate(self):
    return self.estimate
  
  