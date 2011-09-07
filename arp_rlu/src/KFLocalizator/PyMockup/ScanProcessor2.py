# coding=utf-8
import numpy as np
import math
import random
import logging

from BaseClasses import *
from clusterize import *
import ransac
    
class ScanProcessor2:
  def __init__(self):
    self.beacons = []
    self.objects = []
    self.reset()

    self.medianFilterWidth = 3
    self.clusterParams = ClusterizeParams()
  
  def reset(self):
    self.objects = []
    
  def setTrueStaticPositionForDebugOnly(self, x, y, h):
    self.trueX = x
    self.trueY = y
    self.trueH = h
  
  def process(self, scan, tt, xx, yy, hh):
    self.reset()
    
    mf = MedianFilter(self.medianFilterWidth)
    filtScan = mf.compute(scan)
    pc = PointCloud()
    pc.fromScan(filtScan, tt, xx, yy, hh)
    
    vPC = clusterize(pc, self.clusterParams)
    
    
  
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


  def __associate(self):
    pass

  