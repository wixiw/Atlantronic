#!/usr/bin/env python
# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import params_KFLocalizator_Node as params

class OdoNode():
    
  def __init__(self):
    rospy.init_node('OdoSimulator', anonymous=True)
    self.pub = rospy.Publisher("/odovelocity", TwistWithCovarianceStamped)
    rospy.loginfo(rospy.get_name() + " is running")
    
  def spin(self):
    r = rospy.Rate(params.simu_cfg["frequency"])
    while not rospy.is_shutdown():
        t = TwistWithCovarianceStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.twist.twist.linear.x = 0.
        t.twist.twist.linear.y = 0.
        t.twist.twist.angular.z = 0.
        t.twist.covariance[ 0] = params.simu_cfg["sigmaTransOdoVelocity"]  # sigma_xx
        t.twist.covariance[ 7] = params.simu_cfg["sigmaTransOdoVelocity"]  # sigma_yy
        t.twist.covariance[35] = params.simu_cfg["sigmaRotOdoVelocity"]  # sigma_hh
        self.pub.publish(t)
        r.sleep()


if __name__ == '__main__':
  try:
    odonode = OdoNode()
    odonode.spin()    
  except rospy.ROSInterruptException: pass