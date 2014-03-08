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
                      transitions={'succeeded':'PASS1', 'timeout':'end'})
            

            
            # TEST OF ALL MOTION STATES ##########################
            
            smach.StateMachine.add('AmbiOmniDirectOrder2',
                       AmbiOmniDirectOrder2(Pose2D(0.950,0,0) ,
                                           vmax = 1.0),
                      transitions={'succeeded':'OmniDirectOrder2', 'timeout':'end'})
            smach.StateMachine.add('OmniDirectOrder2',
                       OmniDirectOrder2(Pose2D( 0.950, 0.3, 0) ,
                                           vmax = 1.0),
                      transitions={'succeeded':'AmbiOmniDirectOrder2Pass', 'timeout':'end'}) 
    
            smach.StateMachine.add('AmbiOmniDirectOrder2Pass',
                       AmbiOmniDirectOrder2Pass(Pose2D(0.7,0.3,0) ,
                                           vpasse=0.5,
                                           vmax = 1.0),
                      transitions={'succeeded':'OmniDirectOrder2Pass', 'timeout':'end'})         

            smach.StateMachine.add('OmniDirectOrder2Pass',
                       OmniDirectOrder2Pass(Pose2D( 0.7, -0.3, 0) ,
                                           vpasse=0.5,
                                           vmax = 1.0),
                      transitions={'succeeded':'AmbiOpenLoopOrder', 'timeout':'end'})  

            smach.StateMachine.add('AmbiOpenLoopOrder',
                       AmbiOpenLoopOrder(vx = 0.3, 
                                           vy = 0.3, 
                                           vh = 0.3, 
                                           duration=1.0),
                      transitions={'succeeded':'OpenLoopOrder', 'timeout':'end'})  

            smach.StateMachine.add('OpenLoopOrder',
                       OpenLoopOrder(vx = 0.3, 
                                           vy = 0.3, 
                                           vh = 0.3, 
                                           duration=1.0),
                      transitions={'succeeded':'ForwardOrder', 'timeout':'end'}) 
         
            smach.StateMachine.add('ForwardOrder',
                       ForwardOrder(dist = 0.15),
                      transitions={'succeeded':'BackwardOrder', 'timeout':'end'}) 
            smach.StateMachine.add('BackwardOrder',
                       BackwardOrder(dist = 0.15),
                      transitions={'succeeded':'LeftwardOrder', 'timeout':'end'}) 
            smach.StateMachine.add('LeftwardOrder',
                       LeftwardOrder(dist = 0.15),
                      transitions={'succeeded':'RightwardOrder', 'timeout':'end'}) 
            smach.StateMachine.add('RightwardOrder',
                       RightwardOrder(dist = 0.15),
                      transitions={'succeeded':'AmbiTurnOrder', 'timeout':'end'}) 
            
            smach.StateMachine.add('AmbiTurnOrder',
                       AmbiTurnOrder(h = pi/2),
                      transitions={'succeeded':'TurnOrder', 'timeout':'end'}) 
            smach.StateMachine.add('TurnOrder',
                       TurnOrder(h = pi/2),
                      transitions={'succeeded':'Rewind', 'timeout':'end'})      
            smach.StateMachine.add('Rewind',
                       Rewind(rewindDuration = 2.0),
                      transitions={'succeeded':'Move', 'timeout':'end'})   
            
            # SPECIFIC DEBUG TESTS ##########################
            
            smach.StateMachine.add('M1',
                       AmbiOmniDirectOrder2Pass(Pose2D( 0.950, 0, 0) ,
                                           vpasse=0.1,vmax = 0.3),
                      transitions={'succeeded':'waitM2', 'timeout':'end'})
             
            smach.StateMachine.add('waitM2',
                      WaiterState(5),
                      transitions={'timeout':'M2'})
             
                        
            smach.StateMachine.add('M2',
                       AmbiOmniDirectOrder2(Pose2D( 0.950, 0.3,0) ,
                                           vmax = 1.0),
                      transitions={'succeeded':'waitMove', 'timeout':'end'})            
            
            smach.StateMachine.add('waitMove',
                      WaiterState(5),
                      transitions={'timeout':'Move'})
                        
            # TEST OF ALL RANDOM OMNIDIRECT ORDERS ##########################
                        
            smach.StateMachine.add('Move', RandomMove(),
                                   transitions={'succeeded':'Move', 'timeout':'end'}) 

            # TEST OF PASS ##########################      
            smach.StateMachine.add('PASS1',
                       AmbiOmniDirectOrder2Pass(Pose2D( 1.050, 0.5, 0) ,
                                           vpasse=0.3,vmax = 1),
                      transitions={'succeeded':'PASS2', 'timeout':'end'})   
            smach.StateMachine.add('PASS2',
                       AmbiOmniDirectOrder2Pass(Pose2D( 0.750, 0.7, pi/2) ,
                                           vpasse=0.3,vmax = 1),
                      transitions={'succeeded':'PASS3', 'timeout':'end'})   
            smach.StateMachine.add('PASS3',
                       AmbiOmniDirectOrder2Pass(Pose2D( 0.450, 0, pi ) ,
                                           vpasse=0.3,vmax = 1),
                      transitions={'succeeded':'PASS4', 'timeout':'end'})   
            smach.StateMachine.add('PASS4',
                       AmbiOmniDirectOrder2(Pose2D( 0.750, 0, -pi) ,
                                           vmax = 1),
                      transitions={'succeeded':'end', 'timeout':'end'})     
       


class RandomMove(MotionState):
    def __init__(self):
        MotionState.__init__(self)
        #seed = random.randint(0, 1000)
        seed=581
        random.seed(seed)
        rospy.loginfo("------------MOTIONTESTING INIT----------------")
        rospy.loginfo("randomized with seed: %d" % (seed))
        rospy.loginfo("----------------------------------------------")

    def createAction(self):
        self.x = random.uniform(0.4, 1.1)
        self.y = random.uniform(-0.6, 0.6)
        self.theta = random.uniform(-pi, pi)
        self.vmax = random.uniform(0.1, 1)
        self.vpass = random.uniform(0.1, 0.5)
        rospy.loginfo("--- OMNIDIRECTORDER2 ---")
        rospy.loginfo("x =%.3f y=%.3f theta=%.3f"%(self.x, self.y, self.theta))
        rospy.loginfo("vmax =%.3f"%(self.vmax))

        if random.uniform(0, 1)< 0.0 :
            rospy.loginfo("Stop on point")
            self.omnidirect2(self.x, self.y, self.theta, self.vmax)
            #self.omnidirect2(self.x, self.y, self.theta, 1)
        else:
            rospy.loginfo("Pass with v=%.3f"%(self.vpass))
            self.omnidirect2Pass(self.x, self.y, self.theta, self.vpass,self.vmax)

            
            
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
