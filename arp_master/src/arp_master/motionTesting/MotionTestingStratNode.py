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
                      #transitions={'succeeded':'p1', 'timeout':'end'})
                      #transitions={'succeeded':'AmbiOmniDirectOrder2', 'timeout':'end'})
                      transitions={'succeeded':'M1', 'timeout':'end'})
            
            # VIDEO DEMO ##########################

            x1=1.100
            y1=0.350
            x2=0.400
            y2=0.350
            x3=0.400
            y3=-0.350
            x4=1.100
            y4=-0.350
            wait_time=0.1
            

            #carre moisi
            
            
            smach.StateMachine.add('p1',
                       OmniDirectOrder2(x = x1, 
                                           y = 0, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp1', 'timeout':'end'})
             
            smach.StateMachine.add('wp1',
                      WaiterState(wait_time),
                      transitions={'timeout':'p2'})
            
            smach.StateMachine.add('p2',
                       OmniDirectOrder2(x = x1, 
                                           y = 0, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp2', 'timeout':'end'})
             
            smach.StateMachine.add('wp2',
                      WaiterState(wait_time),
                      transitions={'timeout':'p3'})
            
            smach.StateMachine.add('p3',
                       OmniDirectOrder2(x = x1, 
                                           y = y1, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp3', 'timeout':'end'})
             
            smach.StateMachine.add('wp3',
                      WaiterState(wait_time),
                      transitions={'timeout':'p4'})

           
            smach.StateMachine.add('p4',
                       OmniDirectOrder2(x = x1, 
                                           y = y1, 
                                           theta = pi, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp4', 'timeout':'end'})
             
            smach.StateMachine.add('wp4',
                      WaiterState(wait_time),
                      transitions={'timeout':'p5'}) 
            
            smach.StateMachine.add('p5',
                       OmniDirectOrder2(x = x2, 
                                           y = y2, 
                                           theta = pi, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp5', 'timeout':'end'})
             
            smach.StateMachine.add('wp5',
                      WaiterState(wait_time),
                      transitions={'timeout':'p6'})
            
            smach.StateMachine.add('p6',
                       OmniDirectOrder2(x = x2, 
                                           y = y2, 
                                           theta = -pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp6', 'timeout':'end'})
             
            smach.StateMachine.add('wp6',
                      WaiterState(wait_time),
                      transitions={'timeout':'p7'})   
            
            smach.StateMachine.add('p7',
                       OmniDirectOrder2(x = x3, 
                                           y = y3, 
                                           theta = -pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp8', 'timeout':'end'})
             
            smach.StateMachine.add('wp8',
                      WaiterState(wait_time),
                      transitions={'timeout':'p9'})    
            
            smach.StateMachine.add('p9',
                       OmniDirectOrder2(x = x3, 
                                           y = y3, 
                                           theta = pi, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp9', 'timeout':'end'})
             
            smach.StateMachine.add('wp9',
                      WaiterState(wait_time),
                      transitions={'timeout':'p10'})               
            
            smach.StateMachine.add('p10',
                       OmniDirectOrder2(x = x4, 
                                           y = y4, 
                                           theta = pi, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp10', 'timeout':'end'})
             
            smach.StateMachine.add('wp10',
                      WaiterState(wait_time),
                      transitions={'timeout':'p11'})  
            
            smach.StateMachine.add('p11',
                       OmniDirectOrder2(x = x4, 
                                           y = y4, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp11', 'timeout':'end'})
             
            smach.StateMachine.add('wp11',
                      WaiterState(wait_time),
                      transitions={'timeout':'p12'}) 
            
            smach.StateMachine.add('p12',
                       OmniDirectOrder2(x = x1, 
                                           y = y1, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp12', 'timeout':'end'})
             
            smach.StateMachine.add('wp12',
                      WaiterState(wait_time),
                      transitions={'timeout':'p13'})  
            
            # carre translation
            smach.StateMachine.add('p13',
                       OmniDirectOrder2(x = x2, 
                                           y = y2, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p14', 'timeout':'end'})
            
            smach.StateMachine.add('p14',
                       OmniDirectOrder2(x = x3, 
                                           y = y3,
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p15', 'timeout':'end'})

            smach.StateMachine.add('p15',
                       OmniDirectOrder2(x = x4, 
                                           y = y4, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p16', 'timeout':'end'})
            
            smach.StateMachine.add('p16',
                       OmniDirectOrder2(x = x1, 
                                           y = y1, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp16', 'timeout':'end'})
            
            smach.StateMachine.add('wp16',
                      WaiterState(wait_time),
                      transitions={'timeout':'p17'}) 
            
            ## carre avec rotation
            smach.StateMachine.add('p17',
                       OmniDirectOrder2(x = x2, 
                                           y = y2, 
                                           theta = -pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p18', 'timeout':'end'})
            
            smach.StateMachine.add('p18',
                       OmniDirectOrder2(x = x3, 
                                           y = y3, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p19', 'timeout':'end'})
            
            smach.StateMachine.add('p19',
                       OmniDirectOrder2(x = x4, 
                                           y = y4, 
                                           theta = -pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p20', 'timeout':'end'})
            
            smach.StateMachine.add('p20',
                       OmniDirectOrder2(x = x1, 
                                           y = y1, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'wp20', 'timeout':'end'})
            
            smach.StateMachine.add('wp20',
                      WaiterState(wait_time),
                      transitions={'timeout':'p21'}) 
            
            ## carre passage
            smach.StateMachine.add('p21',
                       OmniDirectOrder2Pass(x = x2, 
                                           y = y2, 
                                           theta = -pi/2,
                                           vpasse=0.5, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p22', 'timeout':'end'})
            
            smach.StateMachine.add('p22',
                       OmniDirectOrder2Pass(x = x3, 
                                           y = y3, 
                                           theta = pi/2, 
                                           vpasse=0.5, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p23', 'timeout':'end'})
            
            smach.StateMachine.add('p23',
                       OmniDirectOrder2Pass(x = x4, 
                                           y = y4, 
                                           theta = -pi/2, 
                                           vpasse=0.5, 
                                           vmax = 1.0),
                      transitions={'succeeded':'p24', 'timeout':'end'})
            
            smach.StateMachine.add('p24',
                       OmniDirectOrder2(x = x1, 
                                           y = y1, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'end', 'timeout':'end'})
            
            # TEST OF ALL MOTION STATES ##########################
            
            smach.StateMachine.add('AmbiOmniDirectOrder2',
                       AmbiOmniDirectOrder2(x = 0.950, 
                                           y = 0, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'OmniDirectOrder2', 'timeout':'end'})
            smach.StateMachine.add('OmniDirectOrder2',
                       OmniDirectOrder2(x = 0.950, 
                                           y = 0.3, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'AmbiOmniDirectOrder2Pass', 'timeout':'end'}) 
    
            smach.StateMachine.add('AmbiOmniDirectOrder2Pass',
                       AmbiOmniDirectOrder2Pass(x = 0.7, 
                                           y = 0.3, 
                                           theta = 0, 
                                           vpasse=0.5,
                                           vmax = 1.0),
                      transitions={'succeeded':'OmniDirectOrder2Pass', 'timeout':'end'})         

            smach.StateMachine.add('OmniDirectOrder2Pass',
                       OmniDirectOrder2Pass(x = 0.7, 
                                           y = -0.3, 
                                           theta = 0, 
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
                       AmbiOmniDirectOrder2(x = 0.950, 
                                           y = 0, 
                                           theta = 0, 
                                           vmax = 1.0),
                      transitions={'succeeded':'waitM2', 'timeout':'end'})
             
            smach.StateMachine.add('waitM2',
                      WaiterState(1),
                      transitions={'timeout':'M2'})
             
                        
            smach.StateMachine.add('M2',
                       AmbiOmniDirectOrder2(x = 0.600, 
                                           y = 0, 
                                           theta = pi/2, 
                                           vmax = 1.0),
                      transitions={'succeeded':'waitMove', 'timeout':'end'})            
            
            smach.StateMachine.add('waitMove',
                      WaiterState(1),
                      transitions={'timeout':'Move'})
                        
            # TEST OF ALL RANDOM OMNIDIRECT ORDERS ##########################
                        
            smach.StateMachine.add('Move', RandomMove(),
                                   transitions={'succeeded':'Move', 'timeout':'end'}) 

                        
       


class RandomMove(MotionState):
    def __init__(self):
        MotionState.__init__(self)
        #seed = random.randint(0, 1000)
        seed=221
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

        if random.uniform(0, 1)< 0.2 :
            rospy.loginfo("Stop on point")
            self.omnidirect2(self.x, self.y, self.theta, self.vmax)
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
