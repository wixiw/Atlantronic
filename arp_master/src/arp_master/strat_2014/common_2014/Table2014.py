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
    
    #Torche mobile
    P_YELLOW_MOBILE_TORCH       = Point(0.600,-0.050)
    MOBILE_TORCH_RADIUS  = 0.080
    
    
    P_START_POS         = Pose2D(1.260, 0.550, -5*pi/6)
    P_IN_FRONT_START_AREA   = Pose2D(1.100, 0.400, pi/4)

#Yellow Heat Bot
    P_YELLOW_HEAT_BOT       = Pose2D(1.500, -1.000)
    HEAT_BOT_RADIUS         = 0.250
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
        
        
