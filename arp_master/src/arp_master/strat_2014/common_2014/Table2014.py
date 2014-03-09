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
    #     les points sont defini cote rouge comme toujours
    
    #bouteilles
    #P_BOTTLE_CLOSE      = Point(0.860,-0.900)

    
    # 
    # Memoire des actions
    #
    objectOneEmpty = False

    recallWalls={'RIGHT':(1.500-RobotVierge.FRONT_SIDE.x,"FREE",0),
                 'LEFT':(-1.500+RobotVierge.FRONT_SIDE.x,"FREE",-pi),
                 'UP':("FREE",1.000-RobotVierge.FRONT_SIDE.x,pi/2),
                 'DOWN':("FREE",-1.000+RobotVierge.FRONT_SIDE.x,-pi/2),
                 'FRUITBASKET':("FREE",0.700-RobotVierge.FRONT_SIDE.x,pi/2)
                 }
    
    @staticmethod
    def printStratInfo():
        rospy.loginfo("*********************************************")
        rospy.loginfo("StratInfo :")
        
        rospy.loginfo("")
        rospy.loginfo("*********************************************")
        
        
