#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *

import smach
import rospy


class ShooterNode():
    def __init__(self):
        
        #creation of the node
        rospy.init_node('ShooterNode')
        #recuperation des parametres (on a besoin d'etre un noeud pour ca
        Table2014.getParams()
        Robot2014.getParams()
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm=ShooterMainStateMachine()
    
        #welcome message
        rospy.loginfo("********************************************************")
        rospy.loginfo("Welcome to Advanced Robotics Platform. I am ShooterNode.")
        rospy.loginfo("********************************************************")
    
        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/ShooterNode')
        sis.start()
        sm.execute()
        
        sis.stop()
        
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# this main function is the one called by ros
if __name__ == '__main__':
    try:
        ShooterNode()
    except smach.InvalidTransitionError:
        rospy.loginfo("handling smach.InvalidTransitionError ...")
        rospy.loginfo("Exiting")
        while True:
            pass
    
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass