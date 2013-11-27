#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
import random

from arp_master import *
from arp_master.strat_2014 import *

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
                      SetInitialPosition(0, 0, 0),
                      transitions={'succeeded':'M1', 'timeout':'Debloque'})
            
            
            smach.StateMachine.add('M1',
                       AmbiOmniDirectOrder2(x = 0.6, 
                                           y = 0, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'waitM2', 'timeout':'Debloque'})
             
            smach.StateMachine.add('waitM2',
                      WaiterState(3),
                      transitions={'timeout':'M2'})
             
                        
            smach.StateMachine.add('M2',
                       AmbiOmniDirectOrder2(x = 0.600, 
                                           y = 0, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'waitMove', 'timeout':'Debloque'})            
            
            smach.StateMachine.add('waitMove',
                      WaiterState(3),
                      transitions={'timeout':'Move'})
                        
                                    
            smach.StateMachine.add('Move', RandomMove(),
                                   transitions={'succeeded':'Move', 'timeout':'Debloque'}) 

            
            smach.StateMachine.add('Debloque', Rewind(2.0),
                                   transitions={'succeeded':'end', 'timeout':'end'})  
                        
            smach.StateMachine.add('Wait', WaiterState(2.0),
                                   transitions={'timeout':'end'})           


class RandomMove(MotionState):
    def __init__(self):
        MotionState.__init__(self)
        seed = random.randint(0, 1000)
        random.seed(seed)
        rospy.loginfo("------------MOTIONTESTING INIT----------------")
        rospy.loginfo("randomized with seed: %d" % (seed))
        rospy.loginfo("----------------------------------------------")

    def createAction(self):
        self.x = random.uniform(-1.3, 1.3)
        self.y = random.uniform(-0.8, 0.8)
        self.theta = random.uniform(-pi, pi)
        self.vmax = random.uniform(0.1, 1)
        self.omnidirect2(self.x, self.y, self.theta, self.vmax)
        
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
