#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import smach
import rospy

###########################  TEMPORAL BEHAVIOR

class TestingShooter():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('TestingShooterNode')
        #recuperation des parametres (on a besoin d'etre un noeud pour ca
        Table2014.getParams()
        Robot2014.getParams()
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm=MainStateMachine()
    
        #welcome message
        rospy.loginfo("******************************************************")
        rospy.loginfo("Welcome to Advanced Robotics Platform. I am TestingShooterNode.")
        rospy.loginfo("******************************************************")
    
        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/TestingShooterNode')
        sis.start()
        sm.execute()
        
        sis.stop()
    
############################## MAIN STATE MACHINE CREATION
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('TestingShooter', Shooter(),
                                   transitions={'end':'end','problem':'end'})

   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# this main function is the one called by ros
if __name__ == '__main__':
    try:
        TestingShooter()
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass