#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
import smach_msgs

from CyclicState import CyclicState
from CyclicActionState import CyclicActionState
from Inputs import Inputs
from Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from Table2011 import *
from UtilARD import *

class StartSequence(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('Recal_petit_bord',
                      Recal_petit_bord(),
                      transitions={'succeeded':'Retour_petit_bord','aborted':'problem'})
            
            smach.StateMachine.add('Retour_petit_bord',
                      Retour_petit_bord(),
                      transitions={'succeeded':'Petit_to_Grand','aborted':'problem'})

            smach.StateMachine.add('Petit_to_Grand',
                      Petit_to_Grand(),
                      transitions={'succeeded':'Recal_grand_bord','aborted':'problem'})


            smach.StateMachine.add('Recal_grand_bord',
                      Recal_grand_bord(),
                      transitions={'succeeded':'Retour_grand_bord','aborted':'problem'})


            smach.StateMachine.add('Retour_grand_bord',
                      Retour_grand_bord(),
                      transitions={'succeeded':'Turn_for_match','aborted':'problem'})

            smach.StateMachine.add('Turn_for_match',
                      Turn_for_match(),
                      transitions={'succeeded':'WaitForMatch','aborted':'problem'})


            smach.StateMachine.add('WaitForMatch', WaitForMatch(),
                                   transitions={'start':'gogogo'})
    
            

class Recal_petit_bord(CyclicActionState):
    def createAction(self):
       self.backward(0.300)
      

class Retour_petit_bord(CyclicActionState):
    def createAction(self):
       self.forward(0.300)
     

class Petit_to_Grand(CyclicActionState):
    def createAction(self):
       if Data.color=='red':
           self.cap(0)
       else:
           self.cap(-pi)

class Recal_grand_bord(CyclicActionState):
    def createAction(self):
       self.backward(0.600)

class Retour_grand_bord(CyclicActionState):
    def createAction(self):
       self.forward(0.070)
class Turn_for_match(CyclicActionState):
    def createAction(self):
       if Data.color=='red':
           self.cap(0)
       else:
           self.cap(-pi)
       
       
        
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'start'
        
    def executeOut(self):
        Data.start_time=rospy.get_rostime()
        