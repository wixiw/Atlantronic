#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import os

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.TableVierge import *
from arp_master.strat.util.UtilARD import *
from arp_master.strat.util.WaiterState import WaiterState
from math import pi



class StartSequence(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('SetPower',
                      SetPower(),
                      transitions={'succeeded':'ShowReady','failed':'problem'})
            
            smach.StateMachine.add('ShowReady',
                      ShowReady(),
                      transitions={'succeeded':'TakeStartPosition','aborted':'problem'})

            smach.StateMachine.add('TakeStartPosition',
                      TakeStartPosition(),
                      transitions={'succeeded':'WaitBeforeNext','aborted':'problem'})
            
            smach.StateMachine.add('WaitBeforeNext',
                      WaiterState(1.0),
                      transitions={'done':'WaitForMatch'})
            
            smach.StateMachine.add('WaitForMatch', WaitForMatch(),
                                   transitions={'start':'gogogo'})
    
            
            
class SetPower(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded','failed'])
    
    def executeIn(self):
        self.result = self.enablePower()
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'failed'   
       
       #on tourne un peu avant le match pour confirmer la couleur
class ShowReady(CyclicActionState):
    def createAction(self):
        self.omnicap(AmbiCapRed(-pi+0.8,Data.color).angle)
        
class TakeStartPosition(CyclicActionState):
    def createAction(self):
       self.omnicap(AmbiCapRed(-pi,Data.color).angle)
        
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeIn(self):
        os.system("beep -f 300 -l300 -r3") 
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'start'
        
    def executeOut(self):
        #je libere la vitesse
        self.setVMaxDefault()
        #je note le temps de debut de match
        Data.start_time=rospy.get_rostime()
        
