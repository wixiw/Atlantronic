#!/usr/bin/env python
# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy
import sys
sys.path.append( "./src/KFLocalizator/PyMockup" )

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import *
from arp_rlu.srv import *

import numpy as np
import matplotlib
matplotlib.use('GTKCairo')
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

import math
import KFLocalizator
import params_KFLocalizator_Node as params
from BaseClasses import OdoVelocity
from Scan import Scan

class KFLocalizatorNode():
    
  def __init__(self):
    rospy.init_node('KFLocalizator')
    
    rospy.Subscriber("/scan", LaserScan, self.callbackScan)
    rospy.Subscriber("/odovelocity", TwistWithCovarianceStamped, self.callbackOdo)
    
    self.pub = rospy.Publisher("/KFLocalizator/pose", PoseWithCovarianceStamped)
    
    rospy.Service('initialize', KFLocInit, self.cb_Init)
    rospy.Service('update', Empty, self.update)
    rospy.Service('predict', Empty, self.predict)
      
    self.kfloc = KFLocalizator.KFLocalizator()
    self.initialize(0., 0., 0.)
    
    self.scan = None
    self.odovel = None
    
    self.fig = plt.figure(1)
    plt.ioff()
    plt.hold(True)
    
    self.time = rospy.Time.now()
    rospy.loginfo("start time is %f", self.time.to_sec())
    
    rospy.loginfo(rospy.get_name() + " is running")
    
    
  def initialize(self, initialXPosition, initialYPosition, initialHeading):
    self.kfloc.initialize(rospy.get_time(),100,
                          initialXPosition, 
                          initialYPosition, 
                          initialHeading, 
                          params.kf_cfg["sigmaInitialPosition"], params.kf_cfg["sigmaInitialHeading"],
                          params.kf_cfg["sigmaTransOdoVelocity"], params.kf_cfg["sigmaRotOdoVelocity"], 
                          params.kf_cfg["sigmaLaserRange"], params.kf_cfg["sigmaLaserAngle"], params.kf_cfg["sigmaSegmentHeading"])
    self.kfloc.Nit = params.kf_cfg["iekf_cfg"]["Nit"]
    self.kfloc.threshold = params.kf_cfg["iekf_cfg"]["threshold"]
    self.kfloc.scanproc.maxDistance = params.kf_cfg["scanproc_cfg"]["maxDistance"]
    self.kfloc.scanproc.thresholdRange = params.kf_cfg["scanproc_cfg"]["thresholdRange"]
    
    
  def callbackOdo(self, data):
    self.odovel = OdoVelocity()
    self.odovel.vx = data.twist.twist.linear.x
    self.odovel.vy = data.twist.twist.linear.y
    self.odovel.vh = data.twist.twist.angular.z
    self.sigmaXOdo = data.twist.covariance[ 0]
    self.sigmaYOdo = data.twist.covariance[ 7]
    self.sigmaHOdo = data.twist.covariance[35]


  def predict(self, req):
    if self.odovel is None:
      rospy.logerr("I did not received odo velocity yet")
      return EmptyResponse()
    
    self.time = self.time + rospy.Duration.from_sec(0.01)
    currentT = self.time.to_sec()
    
    rospy.loginfo(rospy.get_name() + " predicting... [time: %f]", currentT)
    self.kfloc.newOdoVelocity(currentT, self.odovel, self.sigmaXOdo, self.sigmaYOdo, self.sigmaHOdo)
    
    self.publishPose()
    
    rospy.loginfo(rospy.get_name() + " predict OK\n")
    return EmptyResponse()
      
      
  def callbackScan(self, data):
    self.scan = Scan(len(data.ranges))
    for i, r in enumerate(data.ranges):
      if r <= data.range_max and r >= data.range_min:
        self.scan.range[i] = r
      else:
        self.scan.range[i] = 0.
      self.scan.theta[i] = data.angle_min + i*data.angle_increment
      self.scan.tt[i]    = data.header.stamp.to_sec() + i*data.time_increment
      
  
  def update(self, req):
    if self.scan is None:
      rospy.logerr("I did not received scan yet")
      return EmptyResponse()
    
    self.time = self.time + rospy.Duration.from_sec(0.01)
    currentT = self.time.to_sec()
    
    rospy.loginfo(rospy.get_name() + " updating... [time: %f]", currentT)
    self.kfloc.newScan(currentT, self.scan)
    
    self.publishPose()
    
    rospy.loginfo(rospy.get_name() + " update OK\n")
    return EmptyResponse()
      
      
  def cb_Init(self, req):
    self.initialize(req.x, req.y, req.h)
    rospy.loginfo(rospy.get_name() + " initialized with x=" + str(req.x) + " y=" + str(req.y) + " h=" + str(req.h))
    return KFLocInitResponse(True)
  
  
  def publishPose(self):
    estim = self.kfloc.getBestEstimate()
    p = PoseWithCovarianceStamped()
    p.header.stamp = rospy.Time.from_sec(estim[0])
    p.pose.pose.position.x = estim[1].xRobot
    p.pose.pose.position.y = estim[1].yRobot
    p.pose.pose.orientation.z = math.sin(estim[1].hRobot/2.)
    p.pose.pose.orientation.w = math.cos(estim[1].hRobot/2.)
    p.pose.covariance[ 0] = estim[1].covariance[0,0]
    p.pose.covariance[ 1] = estim[1].covariance[1,0]
    p.pose.covariance[ 5] = estim[1].covariance[2,0]
    p.pose.covariance[ 6] = estim[1].covariance[0,1]
    p.pose.covariance[ 7] = estim[1].covariance[1,1]
    p.pose.covariance[11] = estim[1].covariance[2,1]
    p.pose.covariance[30] = estim[1].covariance[0,2]
    p.pose.covariance[31] = estim[1].covariance[1,2]
    p.pose.covariance[35] = estim[1].covariance[2,2]
    self.pub.publish(p)
    self.plot(self.scan, estim)

  def plot(self, scan_, estim_):
    x = estim_[1].xRobot
    y = estim_[1].yRobot
    h = estim_[1].hRobot
    
    self.fig.clear()
    ax = plt.subplot(111, aspect='equal')
    
    # table
    ax.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')
    
    xArrowBeg = x
    yArrowBeg = y
    xArrowEnd = 0.1 * np.cos(h)
    yArrowEnd = 0.1 * np.sin(h)
    arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, alpha=0.3, color="blue")
    ax.add_patch(arrow)
    
#    rospy.loginfo("nb of beacons: %d", len(self.kfloc.scanproc.trueBeacons))
    for obj in self.kfloc.scanproc.trueBeacons:
      border = np.arange( 0.0, 2 * np.pi, np.pi / 100.)
      xBalls = np.cos(border) * obj.radius + obj.xCenter
      yBalls = np.sin(border) * obj.radius + obj.yCenter
#      rospy.loginfo("xBeacon=%f  yBeacon=%f  nbPoints=%s", obj.xCenter, obj.yCenter, str(xBalls.shape))
      ax.plot( xBalls, yBalls, '-g')
    
    if scan_ is not None:
      # scan (ray and impacts)
      N = scan_.range.shape[0]
#      rospy.loginfo("N : %d", N)
#      rospy.loginfo("min(scan_.theta): %f", np.min(scan_.theta))
#      rospy.loginfo("max(scan_.theta): %f", np.max(scan_.theta))
#      rospy.loginfo("min(scan_.range): %f", np.min(scan_.range))
#      rospy.loginfo("max(scan_.range): %f", np.max(scan_.range))
#      rospy.loginfo("min(scan_.tt): %f", np.min(scan_.tt))
#      rospy.loginfo("max(scan_.tt): %f", np.max(scan_.tt))
      for i in range(N):
        if scan_.range[-1-i] > 0.:
          xImpact = x + np.cos(h + scan_.theta[-1-i]) * scan_.range[-1-i]
          yImpact = y + np.sin(h + scan_.theta[-1-i]) * scan_.range[-1-i]
          ax.plot( [xImpact] , [yImpact], 'xb' )
          ax.plot( [x, xImpact] , [y, yImpact], '--b' )
        ax.plot( [x] , [y], 'ob' )
        
      # borders
      ax.plot( [x, x + np.cos(h + np.min(scan_.theta))], 
                [y, y + np.sin(h + np.min(scan_.theta))], '-m')
      ax.plot( [x, x + np.cos(h + np.max(scan_.theta))], 
                [y, y + np.sin(h + np.max(scan_.theta))], '-m')
    
    ax.axis([-1.9, 1.9, -1.4, 1.4])
    plt.show()

if __name__ == '__main__':
  try:
    KFLocalizatorNode()
    rospy.spin()
  except rospy.ROSInterruptException: pass