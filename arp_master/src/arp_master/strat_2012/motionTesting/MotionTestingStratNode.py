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
            smach.StateMachine.add('Initialisation', 
                                   Initialisation(),
                                   transitions={'endInitialisation':'SetInitialPosition','failed':'end'}) #=> utilisation de l'etat commun
            smach.StateMachine.add('SetInitialPosition',
                                   SetInitialPosition(0,0,0 ),
                                   transitions={'succeeded':'WaitForStartUnplug','timeout':'end'})            
            smach.StateMachine.add('WaitForStartUnplug', WaitForStartUnplug(),
                                   transitions={'startunplug':'Move1', 'timeout':'end'})
            
            smach.StateMachine.add('Move1', Move1(),
                                   transitions={'succeeded':'Move2', 'timeout':'Debloque'})
            smach.StateMachine.add('Move2', Move2(),
                                   transitions={'succeeded':'Move3', 'timeout':'Debloque'})
            smach.StateMachine.add('Move3', Move3(),
                                   transitions={'succeeded':'Move4', 'timeout':'Debloque'})
            smach.StateMachine.add('Move4', Move4(),
                                   transitions={'succeeded':'Move1', 'timeout':'Debloque'})   
            smach.StateMachine.add('Debloque', Debloque(),
                                   transitions={'succeeded':'Wait', 'timeout':'Debloque'})     
            smach.StateMachine.add('Wait', WaiterState(2.0),
                                   transitions={'timeout':'Move1'})           


class WaitForStartUnplug(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['startunplug'])
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'startunplug'
        
    def executeOut(self):
        self.enablePower()
   

        
class Move1(CyclicActionState):
    def createAction(self):
        self.omnidirect(0.5,0.5,pi/2)
        
class Move2(CyclicActionState):
    def createAction(self):
        self.openloop_cpoint(0,0,0,0,0.5,0,2.0)       

class Move3(CyclicActionState):
    def createAction(self):
        self.omnidirect(Inputs.getx(),Inputs.gety(),0)   

class Move4(CyclicActionState):
    def createAction(self):
        self.omnidirect_cpoint(0.3,0,0,0,-0.4,pi/2)   
        
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