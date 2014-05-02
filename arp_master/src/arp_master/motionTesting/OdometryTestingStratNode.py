#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from std_srvs.srv import *
import random

#from arp_master import *
#from arp_master.strat_2014 import *
from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *

INIT_POS=Pose2D(0.0,0,0)

###########################  TEMPORAL BEHAVIOR

class OdometryTestingStratNode():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('OdometryTestingStratNode')
        # recuperation des parametres)
        #creation of the cadencer for all states
        Data.stateMachineRate = rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
    
        #welcome message
        rospy.loginfo("******************************************************")
        rospy.loginfo("Welcome to the Odometry tester.")
        rospy.loginfo("I will execute sequence when start is unplugged.")
        rospy.loginfo("******************************************************")
        
        #creation of statemachine
        sm = MainStateMachine()
        
        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/SratNode_vierge')
        sis.start()
        sm.execute()
        sis.stop()

    
############################## STATE MACHINE
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', 
                    Strat_Initialisation.UnSecureInitialisation(),
                    transitions={'endInitialisation':'StartSequence', 'failed':'end'})
            smach.StateMachine.add('StartSequence', 
                    Strat_StartSequence.StartSequence(INIT_POS),
                    transitions={'gogogo':'DrivingCalibration', 'problem':'end'})  
            smach.StateMachine.add('LowSpeedRotation',
                      LowSpeedRotation(),
                      transitions={'succeeded':'end', 'failed':'end'})
            smach.StateMachine.add('DrivingCalibration',
                      DrivingCalibration(),
                      transitions={'succeeded':'end', 'failed':'end'})

            
class DrivingCalibration(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        rotation_speed = 1.0
        with self:       
            smach.StateMachine.add('LogInfo',
                       UserDebugTrigger("Mark down the start position, go ahead to 2m."),
                      transitions={'continue':'DisableDrivingPower'}) 
            smach.StateMachine.add('DisableDrivingPower',
                      DisableDrivingPower(),
                      transitions={'succeeded':'PutTurretsToFront', 'timeout':'failed'})
            smach.StateMachine.add('PutTurretsToFront',
                       OpenLoopOrderWithoutTimeout(vx=0.100, vy=0.0, vh=0.0, duration=2),
                      transitions={'succeeded':'LogResult', 'timeout':'failed'})
            smach.StateMachine.add('LogResult',
                       LoggerState("Read RluMonitor.Odometry.attrTotalDistanceRunLeft and compare to expected measure. Repeat with Rigth and Rear"),
                      transitions={'continue':'LogInfo2', 'timeout':'failed'})   
            
            smach.StateMachine.add('LogInfo2',
                       UserDebugTrigger("Mark down the start position, go laterally to 2m."),
                      transitions={'continue':'PutTurretsToLeft'}) 
            smach.StateMachine.add('PutTurretsToLeft',
                       OpenLoopOrderWithoutTimeout(vx=0.000, vy=0.1, vh=0.0, duration=60),
                      transitions={'succeeded':'LogResult2', 'timeout':'failed'})
            smach.StateMachine.add('LogResult2',
                       LoggerState("Read RluMonitor.Odometry.attrTotalDistanceRunLeft and compare to expected measure. Repeat with Rigth and Rear"),
                      transitions={'continue':'LogInfo3', 'timeout':'failed'})  
            smach.StateMachine.add('LogInfo3',
                       UserDebugTrigger("Finished."),
                      transitions={'continue':'succeeded'})          

# Rotation at low speed to check openloop commands ##########################
class LowSpeedRotation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        rotation_speed = 1.0
        with self:       
            smach.StateMachine.add('LogInfo',
                       UserDebugTrigger("Mark down a rotation 0, and place the robot 45deg prior to this position. Prepare a chronometer : tick on 2 consecutive passage on 0."),
                      transitions={'continue':'LowSpeedRot'}) 
            
            smach.StateMachine.add('LowSpeedRot',
                       OpenLoopOrderWithoutTimeout(vx=0.0, vy=0.0, vh=rotation_speed, duration=pi*3.0/rotation_speed),
                      transitions={'succeeded':'LogResult', 'timeout':'failed'})
            
            smach.StateMachine.add('LogResult',
                       LoggerState("Robot should have done a revolution and a half. Compare to theory : " + str.format("{0:.3f}",pi*2.0/rotation_speed) + " seconds."),
                      transitions={'continue':'succeeded', 'timeout':'LogInfo'})


class OpenLoopOrderWithoutTimeout(OpenLoopOrder):
        def __init__(self,vx,vy,vh,duration):
            OpenLoopOrder.__init__(self,vx,vy,vh,duration)
            self.timeout = 100

            
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        OdometryTestingStratNode()
    except smach.InvalidTransitionError:
        rospy.loginfo("handling smach.InvalidTransitionError ...")
        rospy.loginfo("Exiting")
        while True:
            pass
    
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass
