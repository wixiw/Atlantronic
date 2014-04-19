#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.TableVierge import *

import rospy
import smach

from arp_master.util.Data import *
from arp_master.util.RobotVierge import *

#il faut voir ca comme un namespace
class Table2014(TableVierge):
    #
    # Points d'interet
    #     les points sont defini cote jaune comme toujours
    
    #bouteilles
    #P_BOTTLE_CLOSE      = Point(0.860,-0.900)

    P_START_POS         = Pose2D(1.260, 0.550, -5*pi/6)
    
    # 
    # Memoire des actions
    #
    objectOneEmpty = False

    
    
    @staticmethod
    def printStratInfo():
        rospy.loginfo("*********************************************")
        rospy.loginfo("StratInfo :")
        
        rospy.loginfo("")
        rospy.loginfo("*********************************************")
        
        
