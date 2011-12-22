#!/usr/bin/env python
# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy
import sys
sys.path.append( "./src/KFLocalizator/PyMockup" )


import numpy as np
import matplotlib
matplotlib.use('GTKCairo')

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

from sensor_msgs.msg import LaserScan
from std_srvs.srv import *
from arp_rlu.srv import *

import BaseClasses
import Scan
import math
import EnhancedScanProcessor

class TestEnhancedScanProcessor():
    
  def __init__(self):
    rospy.init_node('TestEnhancedScanProcessor', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, self.callbackScan)
    rospy.Service('/TestEnhancedScanProcessor/do', KFLocInit, self.do)
    
    self.stop = False
    self.tt = []
    
#    plt.ioff()
    self.fig = plt.figure(1)
    plt.ioff()
      
    self.scanproc = EnhancedScanProcessor.EnhancedScanProcessor()
    rospy.loginfo(rospy.get_name() + " is running")
    rospy.spin()

  def callbackScan(self, data):
    if self.stop:
      return
    self.scan = Scan.Scan(N = len(data.ranges))
    for i, r in enumerate(data.ranges):
      if r <= data.range_max and r >= data.range_min:
        self.scan.range[i] = r
      else:
        self.scan.range[i] = 0.
      self.scan.theta[i] = data.angle_min + i*data.angle_increment
      self.scan.tt[i]    = data.header.stamp.to_sec() + i*data.time_increment
    self.tt = list(self.scan.tt)
    
      
  def do(self, req):
    self.stop = True
    
    tt = self.tt
    N = len(tt)
    xx = [req.x] * N
    yy = [req.y] * N
    hh = [req.h] * N
    
    rospy.loginfo("N: %d", N)
    rospy.loginfo("min(self.scan.theta): %f", np.min(self.scan.theta))
    rospy.loginfo("max(self.scan.theta): %f", np.max(self.scan.theta))
    rospy.loginfo("min(self.scan.range): %f", np.min(self.scan.range))
    rospy.loginfo("max(self.scan.range): %f", np.max(self.scan.range))
    rospy.loginfo("min(self.scan.tt): %f", np.min(self.scan.tt))
    rospy.loginfo("max(self.scan.tt): %f", np.max(self.scan.tt))
    
#    self.scanproc.do(self.scan, tt, xx, yy, hh)
    self.scanproc.setScan(self.scan)
    self.scanproc.findClusters(tt[-N:], xx[-N:], yy[-N:], hh[-N:])
    
    
    rospy.loginfo("################################")
    rospy.loginfo("Nb found clusters before filtering: %d", len(self.scanproc.objects))
    for i,o in enumerate(self.scanproc.objects):
      rospy.loginfo("Object [%d]  x:%f - y:%f with apparent radius:%f", i, o.xCenter, o.yCenter, o.radius)
    
    self.scanproc.filterClusters()
    
    rospy.loginfo("################################")
    rospy.loginfo("Nb found clusters after filtering: %d", len(self.scanproc.objects))
    for i,o in enumerate(self.scanproc.objects):
      rospy.loginfo("Object [%d]  x:%f - y:%f with apparent radius:%f", i, o.xCenter, o.yCenter, o.radius)
    
    self.scanproc.associateClusters()
    rospy.loginfo("################################")
    rospy.loginfo("Nb recognized beacons: %d", len(self.scanproc.foundBeacons))
    for i,o in enumerate(self.scanproc.foundBeacons):
      rospy.loginfo("Object [%d]  x:%f - y:%f with apparent radius:%f", i, o.xCenter, o.yCenter, o.radius)
      
    self.scanproc.cleanResults()
    
    rospy.loginfo("################################")
    rospy.loginfo("Nb recognized beacons after cleaning: %d", len(self.scanproc.foundBeacons))
    for i,o in enumerate(self.scanproc.foundBeacons):
      rospy.loginfo("Object [%d]  x:%f - y:%f with apparent radius:%f", i, o.xCenter, o.yCenter, o.radius)
    
    # clear
    self.fig.clear()
    plt.subplot(111, aspect='equal')
    
    # table
    plt.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')
    # scan (ray and impacts)
    for i in range(N):
      if self.scan.range[-1-i] > 0.:
        xImpact = xx[-1-i] + np.cos(hh[-1-i] + self.scan.theta[-1-i]) * self.scan.range[-1-i]
        yImpact = yy[-1-i] + np.sin(hh[-1-i] + self.scan.theta[-1-i]) * self.scan.range[-1-i]
        plt.plot( [xImpact] , [yImpact], 'xb' )
        plt.plot( [xx[-1-i], xImpact] , [yy[-1-i], yImpact], '--b' )
      plt.plot( [xx[-1-i]] , [yy[-1-i]], 'ob' )
    for i in range(0, N, 10):
      xArrowBeg = xx[-1-i]
      yArrowBeg = yy[-1-i]
      xArrowEnd = 0.07 * np.cos(hh[-1-i] + self.scan.theta[-1-i])
      yArrowEnd = 0.07 * np.sin(hh[-1-i] + self.scan.theta[-1-i])
#      arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.005, alpha = 0.1, color="grey")
#      plt.add_patch(arrow)
    # borders
#    plt.plot( [xx[-N], xx[-N] + np.cos(hh[-N] + np.min(self.scan.theta))], 
#                   [yy[-N], yy[-N] + np.sin(hh[-N] + np.min(self.scan.theta))], '-m')
#    plt.plot( [xx[-1], xx[-1] + np.cos(hh[-1] + np.max(self.scan.theta))], 
#                   [yy[-1], yy[-1] + np.sin(hh[-1] + np.max(self.scan.theta))], '-m')
    # axis
    plt.axis([-1.9, 1.9, -1.4, 1.4])
    plt.show()
    
    self.stop = False
    return KFLocInitResponse(True)

if __name__ == '__main__':
  try:
    TestEnhancedScanProcessor()
  except rospy.ROSInterruptException: pass