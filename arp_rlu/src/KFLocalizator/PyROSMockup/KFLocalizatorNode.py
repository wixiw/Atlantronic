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

import math
import KFLocalizator
import params_KFLocalizator_Node as params
from BaseClasses import OdoVelocity

class KFLocalizatorNode():
    
  def __init__(self):
    rospy.init_node('KFLocalizator', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, self.update)
    rospy.Subscriber("/odovelocity", TwistWithCovarianceStamped, self.predict)
    self.pub = rospy.Publisher("/KFLocalizator/pose", PoseWithCovarianceStamped)
    rospy.Service('start', Empty, self.start)
    rospy.Service('stop', Empty, self.stop)
    rospy.Service('initialize', KFLocInit, self.cb_Init)
      
    self.kfloc = KFLocalizator.KFLocalizator()
    self.run = False
    self.initialize(0., 0., 0.)
    rospy.loginfo(rospy.get_name() + " is running")
    rospy.spin()
    
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

  def predict(self, data):
    if not self.run:
      return
    rospy.loginfo(rospy.get_name() + " predicting...")
    currentT = data.header.stamp.to_sec()
    ov = OdoVelocity()
    ov.vx = data.twist.twist.linear.x
    ov.vy = data.twist.twist.linear.y
    ov.vh = data.twist.twist.angular.z
    sigmaXOdo_ = data.twist.covariance[ 0]
    sigmaYOdo_ = data.twist.covariance[ 7]
    sigmaHOdo_ = data.twist.covariance[35]
    self.kfloc.newOdoVelocity(currentT, ov, sigmaXOdo_, sigmaYOdo_, sigmaHOdo_)
    self.publishPose()
    rospy.loginfo(rospy.get_name() + " predict OK")
      
  def update(self, data):
    if not self.run:
      return
    
    rospy.loginfo(rospy.get_name() + " update!")
    # convert LaserScan into Scan
    # self.kfloc.newScan(...)
      
  def start(self, req):
    self.run = True
    rospy.loginfo(rospy.get_name() + " start!")
    return EmptyResponse()
    
  def stop(self, req):
    self.run = False
    rospy.loginfo(rospy.get_name() + " stop!")
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

if __name__ == '__main__':
  try:
    KFLocalizatorNode()
  except rospy.ROSInterruptException: pass