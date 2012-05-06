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
        Data.stateMachineRate =rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm=MainStateMachine()
    
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
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'StartSequence', 'failed':'end'})
            smach.StateMachine.add('StartSequence', Strat_StartSequence.StartSequence(0,0,0),
                                   transitions={'gogogo':'Move1','problem':'end'})  
            
            smach.StateMachine.add('Move1', Move1(),
                                   transitions={'succeeded':'Move2', 'timeout':'Debloque'})
            smach.StateMachine.add('Move2', Move2(),
                                   transitions={'succeeded':'Move3', 'timeout':'Debloque'})
            smach.StateMachine.add('Move3', Move3(),
                                   transitions={'succeeded':'Move4', 'timeout':'Debloque'})
            smach.StateMachine.add('Move4', Move4(),
                                   transitions={'succeeded':'Move1', 'timeout':'Debloque'})
                
            smach.StateMachine.add('OpenMove1', OpenMove1(),
                                   transitions={'succeeded':'OpenMove2', 'timeout':'Debloque'})
            smach.StateMachine.add('OpenMove2', OpenMove2(),
                                   transitions={'succeeded':'OpenMove3', 'timeout':'Debloque'})
            smach.StateMachine.add('OpenMove3', OpenMove3(),
                                   transitions={'succeeded':'OpenMove4', 'timeout':'Debloque'})
            smach.StateMachine.add('OpenMove4', OpenMove4(),
                                   transitions={'succeeded':'OpenMove5', 'timeout':'Debloque'})
            smach.StateMachine.add('OpenMove5', OpenMove5(),
                                   transitions={'succeeded':'OpenMove6', 'timeout':'Debloque'})
            smach.StateMachine.add('OpenMove6', OpenMove6(),
                                   transitions={'succeeded':'OpenMove1', 'timeout':'Debloque'})   
            smach.StateMachine.add('Debloque', Debloque(),
                                   transitions={'succeeded':'Wait', 'timeout':'Debloque'})     
            smach.StateMachine.add('Wait', WaiterState(2.0),
                                   transitions={'timeout':'end'})           


class Move1(CyclicActionState):
    def createAction(self):
        #self.forward(1.000)
        #self.cap(-1.57)
        self.omnidirect(1.000, 0.000, 1.57)

class Move2(CyclicActionState):
    def createAction(self):
        #self.backward(1.000)
        #self.cap(0)
        self.omnidirect(0.000, 0.000, 0)
        
class Move3(CyclicActionState):
    def createAction(self):
        self.left(1.000) 
        #self.cap(3.14)
  
class Move4(CyclicActionState):
    def createAction(self):
        self.right(1.000)
        #self.cap(0)
              
class OpenMove1(CyclicActionState):
    def createAction(self):
        self.openloop( x_speed=0.500,y_speed=0.000,theta_speed=0.000,
                              openloop_duration=2.000)
        
class OpenMove2(CyclicActionState):
    def createAction(self):
        self.openloop( x_speed=-0.500,y_speed=0.000,theta_speed=0.000,
                      openloop_duration=2.000)   
        

class OpenMove3(CyclicActionState):
    def createAction(self):
        self.openloop( x_speed=0.000,y_speed=1.000,theta_speed=0.000,
                      openloop_duration=1.800)     

class OpenMove4(CyclicActionState):
    def createAction(self):
        self.openloop( x_speed=0.000,y_speed=-1.000,theta_speed=0.000,
                      openloop_duration=1.800)    
        
class OpenMove5(CyclicActionState):
    def createAction(self):
 #       self.openloop( x_speed=0.000,y_speed=-0.000,theta_speed=1.000,
  #                    openloop_duration=2.000)    
                self.openloop_cpoint( -0.0,0,0,
                                     x_speed=0.000,y_speed=-0.000,theta_speed=1.000,
                      openloop_duration=2.000) 
                  
class OpenMove6(CyclicActionState):
    def createAction(self):
  #      self.openloop( x_speed=0.000,y_speed=-0.000,theta_speed=-1.000,
   #                   openloop_duration=2.000)    
                self.openloop_cpoint( -0.0,0,0,
                                       x_speed=0.000,y_speed=-0.000,theta_speed=-1.000,
                      openloop_duration=2.000)    
        
class Debloque(CyclicActionState):
    def createAction(self):
        self.replay(1.0)  
        
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