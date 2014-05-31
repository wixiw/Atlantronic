#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')


from arp_master import *

from Table2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.actuators import *


####################################################################################################################
#configure software that depends on the color
class SetColor(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeIn(self):
        Data.color=Inputs.getcolor()
        if Data.color=='red':
            Data.adv_color='yellow'
        else:
            Data.adv_color='red'
    
    def executeTransitions(self):
       rospy.loginfo("Execute.")
       if  self.configureColor(Data.color) is True:
            return 'done'
    
    def executeOut(self):
        rospy.loginfo("Color configured.")
      
      
class InitSequence2014(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endInitialisation','failed'])
        with self:
 
            smach.StateMachine.add('WaitForOrocos', 
                                   WaitForOrocos(),
                                   transitions={'deployed':'UnsetStm32Power','timeout':'WaitForOrocos'})

#Begin galere STM32 et hokuyo qui ne s'aiment pas ... => TODO willy a priori a virer si ca marche maintenant que les hokuyo sont sur rs232
            smach.StateMachine.add('UnsetStm32Power',
                       SendStm32PowerCmd(False),
                       transitions={'done':'WaitStm32PowerDown'})
              
            smach.StateMachine.add('WaitStm32PowerDown',
                       WaitStm32PowerCmd(False),
                       transitions={'power_state_reached':'WaiterState','timeout':'UnsetStm32Power'})   
              
            smach.StateMachine.add('WaiterState',
                       WaiterState(3.0),
                       transitions={'timeout':'ResetStm32'})   
            
            smach.StateMachine.add('ResetStm32',
                       ResetStm32(),
                       transitions={'succeeded':'WaitForStartUnplug','failed':'UnsetStm32Power'})                 
              
            smach.StateMachine.add('WaitForStartUnplug', 
                                   WaitForStartUnplug(),
                                   transitions={'startunplug':'InformInitDone', 'timeout':'WaitForStartUnplug'}) 
            smach.StateMachine.add('InformInitDone', 
                                   InformInitialized(),
                                   transitions={'done':'WaitStm32ColorChoice'})
            smach.StateMachine.add('WaitStm32ColorChoice', 
                                   WaitStm32ColorChoice(),
                                   transitions={'color_choice_done':'WaitForStart','timeout':'WaitStm32ColorChoice'})
            smach.StateMachine.add('WaitForStart', 
                                   WaitForStart(),
                                   transitions={'start':'FindSteeringZeros','timeout':'WaitForStart'})
            smach.StateMachine.add('FindSteeringZeros',
                                   InitTurretZeros(),             
                                   transitions={'succeeded':'SetColor', 'problem':'failed'})
            smach.StateMachine.add('SetColor', 
                                   SetColor(),
                                   transitions={'done':'endInitialisation','timeout':'SetColor'})
     
class StartSequence2014(smach.StateMachine):
    def __init__(self,startPosition):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        
        with self:
            smach.StateMachine.add('SetInitialPosition',
                      SetPositionState(Pose2D(1.350,0.500,0)),
                      transitions={'succeeded':'WaitForStartUnPlug', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForStartUnPlug',
                      WaitForStartUnplug(),
                      transitions={'startunplug':'PrepareActuators', 'timeout':'WaitForStartUnPlug'})
            
            smach.StateMachine.add('PrepareActuators',
                      PrepareActuators(),
                      transitions={'prepared':'WaitForStart'})
            
            smach.StateMachine.add('WaitForStart', 
                       WaitForStart(),
                       transitions={'start':'SetFinalPosition','timeout':'SetFinalPosition'})
            
            smach.StateMachine.add('SetFinalPosition',
                      SetPositionState(Pose2D(1.500 - Robot2014.FRONT_SIDE.x , startPosition.y, startPosition.theta)),
                      transitions={'succeeded':'WaitForMatch', 'timeout':'WaitForMatch'})
                        
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'WaitForMatch'})

       
class InitSimu(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endInitialisation','failed'])
        with self:
 
            smach.StateMachine.add('WaitForOrocos', 
                                   WaitForOrocos(),
                                   transitions={'deployed':'FindSteeringZeros','timeout':'WaitForOrocos'})
            smach.StateMachine.add('FindSteeringZeros',
                                   InitTurretZeros(), 
                                   transitions={'succeeded':'ActivateFakeColor', 'problem':'failed'})
            smach.StateMachine.add('ActivateFakeColor', 
                                   ActivateFakeColor("yellow"),
                                   transitions={'done':'Wait'})
            smach.StateMachine.add('Wait', 
                                   WaiterState(0.5),
                                   transitions={'timeout':'SetColor'})
            smach.StateMachine.add('SetColor', 
                                   SetColor(),
                                   transitions={'done':'FakeStart','timeout':'SetColor'})
            smach.StateMachine.add('FakeStart', 
                                   FakeStart(),
                                   transitions={'start':'endInitialisation','timeout':'FakeStart'})
            

class ActivateFakeColor(StaticSendOnTopic):
    def __init__(self, color):
        StaticSendOnTopic.__init__(self, "/Ubiquity/color", StartColor, StartColor(color))


class FakeStart(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeTransitions(self):
        return 'start'
       
    def executeOut(self):
        Data.start_time=rospy.get_rostime()
        

class StartSimu(smach.StateMachine):
    def __init__(self,startPosition):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        
        with self:
            smach.StateMachine.add('SetInitialPosition',
                      SetPositionState(startPosition),
                      transitions={'succeeded':'gogogo', 'timeout':'problem'})

####################################################################################################################
