#!/usr/bin/env python
# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy
import sys
print sys.path
sys.path.append( "./src/KFLocalizator/PyMockup" )
import numpy as np

from sensor_msgs.msg import LaserScan
from std_srvs.srv import *
from arp_rlu.srv import *

import math
import EnhancedScanProcessor

class TestEnhancedScanProcessor():
    
  def __init__(self):
    rospy.init_node('TestEnhancedScanProcessor', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, self.callbackScan)
    rospy.Service('do', KFLocInit, self.do)
      
    self.scanproc = EnhancedScanProcessor.EnhancedScanProcessor()
    rospy.loginfo(rospy.get_name() + " is running")
    rospy.spin()

  def callbackScan(self, data):
    # convert LaserScan into Scan
    self.scan = Scan(len(data.ranges))
    for i, (r, intensity) in enumerate(zip(data.ranges, data.intensities)):
      if r <= data.range_max and r >= data.range_min:
        self.scan.range[i] = r
      else:
        self.scan.range[i] = 0.
      self.scan.theta[i] = data.angle_min + i*data.angle_increment
      self.scan.tt[i]    = data.header.stamp + i*data.time_increment
    self.tt = self.scan.tt
      
  def do(self, req):
    tt = self.tt
    xx = [req.x] * len(tt)
    yy = [req.y] * len(tt)
    hh = [req.h] * len(tt)
    self.scanproc.do(self.scan, tt, xx, yy, hh)
    return KFLocInitResponse(True)

if __name__ == '__main__':
  try:
    TestEnhancedScanProcessor()
  except rospy.ROSInterruptException: pass