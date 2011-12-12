# coding=utf-8
import numpy as np
import math
import random
import logging

import PointCloud
import clusterize
import ransac
import BaseClasses
import BaseMethods

class LaserOnlyLocalizator:
  def __init__(self):
    self.reset()
    
    self.medianFilterWidth = 3
    self.clusterParams = clusterize.ClusterizeParams()
    
    self.refBigLength = math.sqrt( 10 )
    self.refSmallLength = 2.
    
    self.qualityThreshold = 1. / 0.20
    
    self.trueA = np.array([ [1.5], [0.] ])
    self.trueB = np.array([ [-1.5], [1.] ])
    self.trueC = np.array([ [-1.5], [-1.] ])
    
  def reset(self):
    self.estimate = BaseClasses.Estimate()
  
  def process(self, scan_):
    log = logging.getLogger('process')
      
    self.reset()
    
    scan = scan_.copy()
    scan.doMedianFiltering(self.medianFilterWidth)
    
    pc = PointCloud.PointCloud()
    pc.fromScan(scan, [scan.tt[0]], [0], [0], [0])
    
    vPC = clusterize.clusterize(pc, self.clusterParams)
    
    if len(vPC) < 3:
      log.warning("Less than 3 valid clusters found")
      return
    
    # find beacons in candidates
    bestComb = None
    bestQuality = None
    combs = BaseMethods.enumerateCombinations(range(len(vPC)),3)
    for comb in combs:
      quality = self.__evalCombination(vPC, comb)
      if bestComb is None:
        bestComb = comb
        bestQuality = quality
      else:
        if quality > bestQuality:
          bestComb = comb
          bestQuality = quality
          
    if quality < self.qualityThreshold:
      log.warning("Unable to recognize beacons in clusters : quality incorrect")
      return
    
    p = self.__identifyBeacons(vPC, bestComb)
    if p == []:
      log.warning("Unable to recognize beacons in clusters : identification failed")
      return
    A = self.__findCenter(vPC[bestComb[p[0]]])
    B = self.__findCenter(vPC[bestComb[p[1]]])
    C = self.__findCenter(vPC[bestComb[p[2]]])
    
    self.estimate = self.__computeEstimate(A, B, C)
    return
    
  def __computeEstimate(self, A,B,C):
    bary = (A + B + C) / 3.
    trueBary = ( self.trueA + self.trueB + self.trueC ) / 3.
    nC = np.cross( np.vstack( (B - A, np.zero((1,2))) ), np.array([[0.],[0.],[1.]]) ) 
    nA = np.cross( np.vstack( (C - B, np.zero((1,2))) ), np.array([[0.],[0.],[1.]]) ) 
    nB = np.cross( np.vstack( (A - C, np.zero((1,2))) ), np.array([[0.],[0.],[1.]]) )  
    nA = nA / np.linalg.norm(nA)
    nB = nB / np.linalg.norm(nB)
    nC = nC / np.linalg.norm(nC)
    trueNC = np.cross( np.vstack( (self.trueB - self.trueA, np.zero((1,2))) ), np.array([[0.],[0.],[1.]]) ) 
    trueNA = np.cross( np.vstack( (self.trueC - self.trueB, np.zero((1,2))) ), np.array([[0.],[0.],[1.]]) ) 
    trueNB = np.cross( np.vstack( (self.trueA - self.trueC, np.zero((1,2))) ), np.array([[0.],[0.],[1.]]) )
    trueNA = trueNA / np.linalg.norm(trueNA)
    trueNB = trueNB / np.linalg.norm(trueNB)
    trueNC = trueNC / np.linalg.norm(trueNC)
    
    angleA = math.atan2( np.cross(nA, trueNA)[2,0] , np.vdot(nA, trueNA))
    angleB = math.atan2( np.cross(nB, trueNB)[2,0] , np.vdot(nB, trueNB))
    angleC = math.atan2( np.cross(nC, trueNC)[2,0] , np.vdot(nC, trueNC))
    h = (angleA + angleB + angleC) / 3.
    
    x = trueBary[0,0] + bary[0,0] * math.cos(h) - bary[1,0] * math.sin(h) 
    y = trueBary[1,0] + bary[0,0] * math.sin(h) + bary[1,0] * math.cos(h)
    
    estim = BaseClasses.Estimate()
    estim.xRobot = x
    estim.yRobot = y
    estim.hRobot = h
    estim.covariance = np.diag((0.05, 0.05, 0.01))
    # todo : compute better covariance
    
    return estim
    
  def __identifyBeacons(self, vPC, bestComb):
    perms = BaseMethods.enumeratePermutations(range(3))
    for p in perms:
      A = self.__findCenter(vPC[bestComb[p[0]]])
      B = self.__findCenter(vPC[bestComb[p[1]]])
      C = self.__findCenter(vPC[bestComb[p[2]]])
      bary = (A + B + C) / 3.
      vA = np.vstack( (A - bary, np.zero((1,2))) )
      vB = np.vstack( (B - bary, np.zero((1,2))) )
      vC = np.vstack( (C - bary, np.zero((1,2))) )
      # check order
      if np.linalg.norm(np.cross(vA, vB)) < 0:
        continue
      if np.linalg.norm(np.cross(vB, vC)) < 0:
        continue
      if np.linalg.norm(np.cross(vC, vA)) < 0:
        continue
      # check if first beacon is first PC
      if np.linalg.norm(C - B) > (self.refBigLength + self.refSmallLength) / 2.:
        continue
      return p
    return []
          
        
  def __evalCombination(self, vPC, comb):
    log = logging.getLogger('__evalCombination')
    A = self.__findCenter(vPC[comb[0]])
    B = self.__findCenter(vPC[comb[1]])
    C = self.__findCenter(vPC[comb[2]])
    ll = []
    ll.append( np.linalg.norm(B - A) )
    ll.append( np.linalg.norm(C - A) )
    ll.append( np.linalg.norm(C - B) )
    maxll = np.max(ll)
    medll = BaseMethods.getMedian(ll)
    minll = np.min(ll)
    log.debug("max lenght = %f",maxll)
    log.debug("med lenght = %f",medll)
    log.debug("min lenght = %f",minll)
    invquality = (maxll - self.refBigLength)**2 + (medll - self.refBigLength)**2 + (minll - self.refSmallLength)**2 
    return 1./ invquality
      
      
  def __findCenter(self, pc):
    return np.mean(pc.points[0:2,0:], 1)
  
  
  def getEstimate(self):
    return self.estimate
  
  