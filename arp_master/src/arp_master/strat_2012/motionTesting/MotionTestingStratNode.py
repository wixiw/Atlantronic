#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

###########################  TEMPORAL BEHAVIOR

class MotionTestingStratNode():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('MotionTestingStratNode')
        # recuperation des parametres)
        #creation of the cadencer for all states
        Data.stateMachineRate = rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm = MainStateMachine()
    
        #welcome message
        rospy.loginfo("******************************************************")
        rospy.loginfo("Welcome to the Motion tester.")
        rospy.loginfo("I will execute sequence when start is unplugged.")
        rospy.loginfo("******************************************************")
        
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
            smach.StateMachine.add('Initialisation', Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'StartSequence', 'failed':'end'})
            smach.StateMachine.add('StartSequence', Strat_StartSequence.StartSequence(0, 0, 0),
                                   transitions={'gogogo':'SetInitialPosition', 'problem':'end'})  
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(0,0,0),
                      transitions={'succeeded':'Forward', 'timeout':'Debloque'})
                        
            smach.StateMachine.add('Forward', AmbiOmniDirectOrder2(0.8,-0.7,0, vmax=0.3),
                                   transitions={'succeeded':'Wait', 'timeout':'Debloque'}) 

            
            smach.StateMachine.add('Debloque', Replay(2.0),
                                   transitions={'succeeded':'end','timeout':'end'})  
                        
            smach.StateMachine.add('Wait', WaiterState(2.0),
                                   transitions={'timeout':'end'})           

        
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        MotionTestingStratNode()
    except smach.InvalidTransitionError:
        rospy.loginfo("handling smach.InvalidTransitionError ...")
        rospy.loginfo("Exiting")
        pass
    
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass
