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
                      SetInitialPosition(0.750, 0, 0),
                      #transitions={'succeeded':'M1', 'timeout':'Debloque'})
                      transitions={'succeeded':'AmbiOmniDirectOrder2', 'timeout':'Debloque'})
            
            #les ordres a tester

            
            # TEST OF ALL MOTION STATES ##########################
            
            smach.StateMachine.add('AmbiOmniDirectOrder2',
                       AmbiOmniDirectOrder2(x = 0.950, 
                                           y = 0, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'OmniDirectOrder2', 'timeout':'Debloque'})
            smach.StateMachine.add('OmniDirectOrder2',
                       OmniDirectOrder2(x = 0.950, 
                                           y = 0.3, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'AmbiOmniDirectOrder2Pass', 'timeout':'Debloque'}) 
    
            smach.StateMachine.add('AmbiOmniDirectOrder2Pass',
                       AmbiOmniDirectOrder2Pass(x = 0.7, 
                                           y = 0.3, 
                                           theta = 0, 
                                           vpasse=0.5,
                                           vmax = 1.0),
                      transitions={'succeeded':'OmniDirectOrder2Pass', 'timeout':'Debloque'})         

            smach.StateMachine.add('OmniDirectOrder2Pass',
                       OmniDirectOrder2Pass(x = 0.7, 
                                           y = -0.3, 
                                           theta = 0, 
                                           vpasse=0.5,
                                           vmax = 1.0),
                      transitions={'succeeded':'AmbiOpenLoopOrder', 'timeout':'Debloque'})  

            smach.StateMachine.add('AmbiOpenLoopOrder',
                       AmbiOpenLoopOrder(vx = 0.3, 
                                           vy = 0.3, 
                                           vh = 0.3, 
                                           duration=1.0),
                      transitions={'succeeded':'OpenLoopOrder', 'timeout':'Debloque'})  

            smach.StateMachine.add('OpenLoopOrder',
                       OpenLoopOrder(vx = 0.3, 
                                           vy = 0.3, 
                                           vh = 0.3, 
                                           duration=1.0),
                      transitions={'succeeded':'ForwardOrder', 'timeout':'Debloque'}) 
         
            smach.StateMachine.add('ForwardOrder',
                       ForwardOrder(dist = 0.15),
                      transitions={'succeeded':'BackwardOrder', 'timeout':'Debloque'}) 
            smach.StateMachine.add('BackwardOrder',
                       BackwardOrder(dist = 0.15),
                      transitions={'succeeded':'LeftwardOrder', 'timeout':'Debloque'}) 
            smach.StateMachine.add('LeftwardOrder',
                       LeftwardOrder(dist = 0.15),
                      transitions={'succeeded':'RightwardOrder', 'timeout':'Debloque'}) 
            smach.StateMachine.add('RightwardOrder',
                       RightwardOrder(dist = 0.15),
                      transitions={'succeeded':'AmbiTurnOrder', 'timeout':'Debloque'}) 
            
            smach.StateMachine.add('AmbiTurnOrder',
                       AmbiTurnOrder(h = pi/2),
                      transitions={'succeeded':'TurnOrder', 'timeout':'Debloque'}) 
            smach.StateMachine.add('TurnOrder',
                       TurnOrder(h = pi/2),
                      transitions={'succeeded':'Rewind', 'timeout':'Debloque'})      
            smach.StateMachine.add('Rewind',
                       Rewind(rewindDuration = 2.0),
                      transitions={'succeeded':'Move', 'timeout':'Debloque'})   
            
            # SPECIFIC TESTS
            
            smach.StateMachine.add('M1',
                       AmbiOmniDirectOrder2(x = 0.950, 
                                           y = 0, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'waitM2', 'timeout':'Debloque'})
             
            smach.StateMachine.add('waitM2',
                      WaiterState(1),
                      transitions={'timeout':'M2'})
             
                        
            smach.StateMachine.add('M2',
                       AmbiOmniDirectOrder2(x = 0.600, 
                                           y = 0, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'waitMove', 'timeout':'Debloque'})            
            
            smach.StateMachine.add('waitMove',
                      WaiterState(1),
                      transitions={'timeout':'Move'})
                        
            # TEST OF ALL RANDOM OMNIDIRECT ORDERS ##########################
                        
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
        #seed=133
        random.seed(seed)
        rospy.loginfo("------------MOTIONTESTING INIT----------------")
        rospy.loginfo("randomized with seed: %d" % (seed))
        rospy.loginfo("----------------------------------------------")

    def createAction(self):
        self.x = random.uniform(0.4, 1.1)
        self.y = random.uniform(-0.6, 0.6)
        self.theta = random.uniform(-pi, pi)
        self.vmax = random.uniform(0.1, 1)
        if random.uniform(0.0,1.0)< 0.3 :
            self.omnidirect2(self.x, self.y, self.theta, self.vmax)
        else:
            self.omnidirect2Pass(self.x, self.y, self.theta, self.vmax/2.0,self.vmax)
        
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
