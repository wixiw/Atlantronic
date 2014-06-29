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
from arp_master.strat_2014 import *

INIT_POS=Pose2D(0.750,0,0)


###########################  TEMPORAL BEHAVIOR

print "-----DEFINITION MotionTestingStratNode"

#for name in dir():
#           print ">>>>> " , name
            
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
            smach.StateMachine.add('Initialisation', InitStates2014.InitSequence2014(),
                                   transitions={'endInitialisation':'StartSequence', 'failed':'end'})
            smach.StateMachine.add('StartSequence', Strat_StartSequence.StartSequence(Pose2D(INIT_POS.x, INIT_POS.y, INIT_POS.theta)),
                                   transitions={'gogogo':'SetInitialPosition', 'problem':'end'})  
            
            smach.StateMachine.add('SetInitialPosition',
                      SetPositionState(Pose2D(INIT_POS.x, INIT_POS.y, INIT_POS.theta)),
                      transitions={'succeeded':'Move', 'timeout':'end'})
            

            
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
                       ForwardOrder(dist = 0.15,vmax=0.3),
                      transitions={'succeeded':'BackwardOrder', 'timeout':'end'}) 
            smach.StateMachine.add('BackwardOrder',
                       BackwardOrder(dist = 0.15,vmax=0.3),
                      transitions={'succeeded':'LeftwardOrder', 'timeout':'end'}) 
            smach.StateMachine.add('LeftwardOrder',
                       LeftwardOrder(dist = 0.15,vmax=0.3),
                      transitions={'succeeded':'RightwardOrder', 'timeout':'end'}) 
            smach.StateMachine.add('RightwardOrder',
                       RightwardOrder(dist = 0.15,vmax=0.3),
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
                       AmbiOmniDirectOrder2(Pose2D(1.0,0.4,-pi/2) ,
                                           vmax = 1.0),
                      transitions={'succeeded':'M2', 'timeout':'end'})
                        
            smach.StateMachine.add('M2',
                       AmbiOmniDirectOrder2(Pose2D(1.0,-0.4,-pi/2),
                                           vmax = 1.0),
                      transitions={'succeeded':'waitMove', 'timeout':'end'})
                                               
            smach.StateMachine.add('waitMove',
                      WaiterState(1),
                      transitions={'timeout':'M1'})
                        
            # TEST OF ALL RANDOM OMNIDIRECT ORDERS ##########################
                        
            smach.StateMachine.add('Move', RandomMove(),
                                   transitions={'succeeded':'Move','timeout':'end'}) 
            

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
       
            # TEST OF RECAL ##########################  

            smach.StateMachine.add('RECAL0',
                       AmbiOmniDirectOrder2(Pose2D( 1.350, 0, 0) ,
                                           vmax = 0.3),
                      transitions={'succeeded':'RECAL1', 'timeout':'end'})     
            
            smach.StateMachine.add('RECAL1',
                       AmbiRecalOnBorderYellow('RIGHT'),
                      transitions={'recaled':'RECAL3','non-recaled':'end' ,'problem':'end'})
            
            smach.StateMachine.add('RECAL3',
                       AmbiOmniDirectOrder2Pass(Pose2D( 1.150, 0, pi/6) ,
                                           vpasse=0.3,vmax = 0.3),
                      transitions={'succeeded':'RECAL4', 'timeout':'end'})  
            
            smach.StateMachine.add('RECAL4',
                       AmbiOmniDirectOrder2(Pose2D( 0.900, 0.900, pi/2) ,
                                           vmax = 0.3),
                      transitions={'succeeded':'RECAL5', 'timeout':'end'}) 
            
            smach.StateMachine.add('RECAL5',
                       AmbiRecalOnBorderYellow('UP'),
                      transitions={'recaled':'RECAL6','non-recaled':'end' ,'problem':'end'})   
                        
            smach.StateMachine.add('RECAL6',
                       AmbiOmniDirectOrder2Pass(Pose2D( 0.900, 0.700, pi/3) ,
                                           vpasse=0.3,vmax = 0.3),
                      transitions={'succeeded':'RECAL0', 'timeout':'end'}) 
            
            
            #### Test Accel
            smach.StateMachine.add('TestAccelFront',
                       AmbiOmniDirectOrder2(Pose2D( 1.300, 0, 0) ,
                                           vmax = 0.7),
                      transitions={'succeeded':'TestAccelRear', 'timeout':'end'})   
            smach.StateMachine.add('TestAccelRear',
                       AmbiOmniDirectOrder2(Pose2D( 0.350, 0, 0) ,
                                           vmax = 0.7),
                      transitions={'succeeded':'TestAccelFront', 'timeout':'end'})   


class RandomMove(MotionState):
    def __init__(self):
        MotionState.__init__(self)
        #seed = random.randint(0, 1000)
        seed=761
        random.seed(seed)
        rospy.loginfo("------------MOTIONTESTING INIT----------------")
        rospy.loginfo("randomized with seed: %d" % (seed))
        rospy.loginfo("----------------------------------------------")



    def createAction(self):
        self.x = random.uniform(0.4, 1.1)
        self.y = random.uniform(-0.6, 0.6)
        self.theta = random.uniform(-pi,pi)
        
        self.vmax = random.uniform(0.1, 1)
        self.vpass = random.uniform(0.05, 1)
        self.movetype = random.uniform(0, 1)
        

        
        if self.movetype < 0.2 :
            rospy.loginfo("------ BFCap ------")
            rospy.loginfo("on current position, theta=%.1f rad"%(self.theta))
            self.cap(self.theta)
        elif self.movetype > 0.6 :
            rospy.loginfo("--- OMNIDIRECTORDER2 ---")
            rospy.loginfo("x =%.3f y=%.3f theta=%.1f vmax =%.1f"%(self.x, self.y, self.theta, self.vmax))
            self.omnidirect2(self.x, self.y, self.theta, self.vmax)
        else:
            rospy.loginfo("--- OMNIDIRECTORDER2 PASS ---")
            rospy.loginfo("x =%.3f y=%.3f theta=%.1f vmax =%.1f Pass with v=%.3f"%(self.x, self.y, self.theta, self.vmax,self.vpass))
            self.omnidirect2Pass(self.x, self.y, self.theta, self.vpass,self.vmax)


    def execute(self,userdata):
        return MotionState.execute(self,userdata)


            
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        MotionTestingStratNode()
    except smach.InvalidTransitionError:
        rospy.loginfo("handling smach.InvalidTransitionError ...")
        rospy.loginfo("Exiting")
        while True:
            pass
    
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass
