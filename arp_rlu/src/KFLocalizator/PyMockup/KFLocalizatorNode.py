#!/usr/bin/env python
# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_srvs.srv import *
import KFLocalizator
import params_KFLocalizator_Node as params

class KFLocalizatorNode():
    
  def __init__(self):
    rospy.init_node('KFLocalizator', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, self.update)
    rospy.Subscriber("/odovelocity", TwistWithCovarianceStamped, self.predict)
    rospy.Service('start', Empty, self.start)
    rospy.Service('stop', Empty, self.stop)
      
    self.kfloc = KFLocalizator.KFLocalizator()
    self.run = False
    rospy.spin()
    
  def initialize(self, initialXPosition, initialYPosition, initialHeading):
    kfloc.initialize(0.,100,
                     initialXPosition, 
                     initialYPosition, 
                     initialHeading, 
                     params.kf_cfg["sigmaInitialPosition"], params.kf_cfg["sigmaInitialHeading"],
                     params.kf_cfg["sigmaTransOdoVelocity"], params.kf_cfg["sigmaRotOdoVelocity"], 
                     params.kf_cfg["sigmaLaserRange"], params.kf_cfg["sigmaLaserAngle"], params.kf_cfg["sigmaSegmentHeading"])
    kfloc.Nit = params.kf_cfg["iekf_cfg"]["Nit"]
    kfloc.threshold = params.kf_cfg["iekf_cfg"]["threshold"]
    kfloc.scanproc.maxDistance = params.kf_cfg["scanproc_cfg"]["maxDistance"]
    kfloc.scanproc.thresholdRange = params.kf_cfg["scanproc_cfg"]["thresholdRange"]

  def predict(self, data):
    if not self.run:
      return
    
    rospy.loginfo(rospy.get_name() + " predict!")
    # convert TwistWithCovarianceStamped into OdoVelocity
    # kfloc.newOdoVelocity(...)
      
  def update(self, data):
    if not self.run:
      return
    
    rospy.loginfo(rospy.get_name() + " update!")
    # convert LaserScan into Scan
    # kfloc.newScan(...)
      
  def start(self, req):
    self.run = True
    rospy.loginfo(rospy.get_name() + " start!")
    
  def stop(self, req):
    self.run = False
    rospy.loginfo(rospy.get_name() + " stop!")
      


if __name__ == '__main__':
  try:
    KFLocalizatorNode()
  except rospy.ROSInterruptException: pass