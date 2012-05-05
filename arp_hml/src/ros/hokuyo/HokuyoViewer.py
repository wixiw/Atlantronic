#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_hml')
import rospy
from sensor_msgs.msg import LaserScan
from arp_hml.srv import *
import numpy as np
import matplotlib.pyplot as plt

class HokuyoViewer():
    
    def __init__(self):
        rospy.init_node('HokuyoViewer', anonymous=True)
        rospy.Subscriber("/top_scan", LaserScan, self.callback)
        self.goon = True
        while self.goon is True:
            rospy.sleep(1.0)
   
        

    def callback(self, data):
        if self.goon is True:
            self.plot(data)
            self.goon = False;
        

    def plot( self, scan):
        rospy.loginfo(rospy.get_name() + "plot.")  
        fig = plt.figure()
        axe = fig.add_subplot(111, aspect='equal')
        for i in range(len(scan.ranges)):
            if scan.ranges[i] > scan.range_min and scan.ranges[i] < scan.range_max:
                xImpact = np.cos(scan.angle_min + i*scan.angle_increment) * scan.ranges[-1-i]
                yImpact = np.sin(scan.angle_min + i*scan.angle_increment)  * scan.ranges[-1-i]
                axe.plot( [xImpact] , [yImpact], 'xb' )
                axe.plot( [0] , [0], 'ob' )
                #axe.plot( [0, xImpact] , [0, yImpact], '--b' )
        axe.axis([0, 3.5, -3.5, 3.5])
        plt.show()

if __name__ == '__main__':
    try:
        HokuyoViewer()
    except rospy.ROSInterruptException: pass
    
    


    
