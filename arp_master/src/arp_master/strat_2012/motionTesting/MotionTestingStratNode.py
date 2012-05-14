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
            
            smach.StateMachine.add('Prepare', Prepare(),
                                   transitions={'succeeded':'SetInitialPosition', 'timeout':'Debloque'})
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(0,0,0),
                      transitions={'succeeded':'Forward', 'timeout':'Debloque'})
                        
            smach.StateMachine.add('Forward', Forward(),
                                   transitions={'succeeded':'Turn', 'timeout':'Debloque'}) 
            smach.StateMachine.add('Turn', Turn(),
                                   transitions={'succeeded':'Move1', 'timeout':'Debloque'})
            smach.StateMachine.add('WaitBack', WaiterState(2.0),
                                   transitions={'timeout':'Forward'})                 
                                    
            smach.StateMachine.add('Move1', Move1(),
                                   transitions={'succeeded':'Wait1', 'timeout':'Debloque'})
            smach.StateMachine.add('Wait1', WaiterState(2.0),
                                   transitions={'timeout':'Move2'})   
            smach.StateMachine.add('Move2', Move2(),
                                   transitions={'succeeded':'Wait2', 'timeout':'Debloque'})
            smach.StateMachine.add('Wait2', WaiterState(2.0),
                                   transitions={'timeout':'Setv2'}) 
            smach.StateMachine.add('Setv2', SetVMaxState(-1.0),
                                   transitions={'succeeded':'Move3','timeout':'end'})   
            smach.StateMachine.add('Move3', Move3(),
                                   transitions={'succeeded':'Wait3', 'timeout':'Debloque'})
            smach.StateMachine.add('Wait3', WaiterState(2.0),
                                   transitions={'timeout':'Setv1'})
            smach.StateMachine.add('Setv1', SetVMaxState(0.2),
                                   transitions={'succeeded':'Move4','timeout':'end'})   
            smach.StateMachine.add('Move4', Move4(),
                                   transitions={'succeeded':'Wait4', 'timeout':'Debloque'})
            smach.StateMachine.add('Wait4', WaiterState(2.0),
                                   transitions={'timeout':'Move1'})    
                           
            smach.StateMachine.add('BMove1', Move4(),
                                   transitions={'succeeded':'BMove2', 'timeout':'Debloque'})
            smach.StateMachine.add('BMove2', Move3(),
                                   transitions={'succeeded':'BMove3', 'timeout':'Debloque'})
            smach.StateMachine.add('BMove3', Move2(),
                                   transitions={'succeeded':'BMove4', 'timeout':'Debloque'})
            smach.StateMachine.add('BMove4', Move1(),
                                   transitions={'succeeded':'BMove1', 'timeout':'Debloque'})
      
            smach.StateMachine.add('Debloque', Replay(1.0),
                                   transitions={'succeeded':'Wait', 'timeout':'Debloque'})     
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
        
class Turn(CyclicActionState):
    def createAction(self):
        self.cap(pi/2)

#--------------------------------------------------------------------------------------------------
class Move1(CyclicActionState):
    def createAction(self):
        #self.omnidirect(0.800, 0.550, -pi/2)
        self.omnidirect(0.800, 0.550, 0)
        

class Move2(CyclicActionState):
    def createAction(self):
        self.omnidirect(-0.800, 0.550, 0,0.3)
        #self.cap(0)
        
class Move3(CyclicActionState):
    def createAction(self):
        self.omnidirect(0.0, 0.550, -pi/2)
        #self.openloop(x_speed=0.400, y_speed=0.000, theta_speed=0.000,
        #              openloop_duration=2.000)  
  
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
