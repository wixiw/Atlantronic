#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from math import *

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
            
            smach.StateMachine.add('Prepare', Prepare(),
                                   transitions={'succeeded':'SetInitialPosition', 'timeout':'Debloque'})
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(0,0,0),
                      transitions={'succeeded':'Forward', 'timeout':'Debloque'})
                        
            smach.StateMachine.add('Forward', AmbiOmniDirectOrder2(0.5,-0.2,pi/2, vmax=0.3),
                                   transitions={'succeeded':'Move1', 'timeout':'Debloque'}) 
             
                                    
            smach.StateMachine.add('Move1', AmbiOmniDirectOrder2(0.800,0.500,pi/4, vmax=0.3),
                                   transitions={'succeeded':'Move2', 'timeout':'Debloque'}) 
            smach.StateMachine.add('Move2', AmbiOmniDirectOrder2(-0.800,0.500,0, vmax=0.3),
                                   transitions={'succeeded':'Move1', 'timeout':'Debloque'})
            
            smach.StateMachine.add('Debloque', Replay(2.0),
                                   transitions={'succeeded':'end','timeout':'end'})  
                        
            smach.StateMachine.add('Wait', WaiterState(2.0),
                                   transitions={'timeout':'end'})           

#--------------------------------------------------------------------------------------------------

class Prepare(CyclicActionState):
    def createAction(self):
        self.openloop(x_speed=0.100, y_speed=0.000, theta_speed=0.000,
                              openloop_duration=0.500)
                              
class Forward(CyclicActionState):
    def createAction(self):
        #self.forward(1.000)
        #self.cap(-1.57)
        self.omnidirect_cpoint(0.0,0.0,0.0,
                               0.000, 0.550, 0)
        


#--------------------------------------------------------------------------------------------------


        


  
class Move4(CyclicActionState):
    def createAction(self):
        self.omnidirect(0.4, 0.550, 0)
        #self.cap(-pi/2)
        #self.openloop(x_speed=-0.400, y_speed=0.000, theta_speed=0.000,
        #              openloop_duration=2.000) 
              
class OpenMove1(CyclicActionState):
    def createAction(self):
        self.openloop(x_speed=0.500, y_speed=0.000, theta_speed=0.000,
                              openloop_duration=2.000)
        
class OpenMove2(CyclicActionState):
    def createAction(self):
        self.openloop(x_speed= -0.200, y_speed=0.000, theta_speed=0.000,
                      openloop_duration=10.000)   
        

class OpenMove3(CyclicActionState):
    def createAction(self):
        self.openloop(x_speed=0.000, y_speed=1.000, theta_speed=0.000,
                      openloop_duration=1.800)     

class OpenMove4(CyclicActionState):
    def createAction(self):
        self.openloop(x_speed=0.000, y_speed= -1.000, theta_speed=0.000,
                      openloop_duration=1.800)    
        
class OpenMove5(CyclicActionState):
    def createAction(self):
 #       self.openloop( x_speed=0.000,y_speed=-0.000,theta_speed=1.000,
  #                    openloop_duration=2.000)    
                self.openloop_cpoint(-0.0, 0, 0,
                                     x_speed=0.000, y_speed= -0.000, theta_speed=1.000,
                      openloop_duration=2.000) 
                  
class OpenMove6(CyclicActionState):
    def createAction(self):
  #      self.openloop( x_speed=0.000,y_speed=-0.000,theta_speed=-1.000,
   #                   openloop_duration=2.000)    
                self.openloop_cpoint(-0.0, 0, 0,
                                       x_speed=0.000, y_speed= -0.000, theta_speed= -1.000,
                      openloop_duration=2.000)    
        
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
