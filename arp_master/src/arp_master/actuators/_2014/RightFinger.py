#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *
            
   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# this main function is the one called by ros
if __name__ == '__main__':
    try:
        #creation of the node
        rospy.init_node('RightFinger')
        #recuperation des parametres (on a besoin d'etre un noeud pour ca
        Table2014.getParams()
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(10)
        #creation of statemachine
        sm=FingerMainStateMachine("Right")
    
        #welcome message
        rospy.loginfo("*** RightFinger ***")

        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        # sis = smach_ros.IntrospectionServer('strat_server', sm, '/RightFinger')
        # sis.start()
        sm.execute()
        # sis.stop()
        
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass        
        