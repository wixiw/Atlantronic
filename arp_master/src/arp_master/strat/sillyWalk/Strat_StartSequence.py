#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
import smach_msgs

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class StartSequence(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('Recal_hor_bord',
                      Recal_hor_bord(),
                      transitions={'succeeded':'SetPos_hor_bord','aborted':'problem'})
            
            smach.StateMachine.add('SetPos_hor_bord',
                      SetPos_hor_bord(),
                      transitions={'done':'Retour_hor_bord'})
         
            smach.StateMachine.add('Retour_hor_bord',
                      Retour_hor_bord(),
                      transitions={'succeeded':'hor_to_vert','aborted':'problem'})

            smach.StateMachine.add('hor_to_vert',
                      hor_to_vert(),
                      transitions={'succeeded':'Recal_vert_bord','aborted':'problem'})


            smach.StateMachine.add('Recal_vert_bord',
                      Recal_vert_bord(),
                      transitions={'succeeded':'SetPos_vert_bord','aborted':'problem'})

            smach.StateMachine.add('SetPos_vert_bord',
                      SetPos_vert_bord(),
                      transitions={'done':'Retour_vert_bord'})

            smach.StateMachine.add('Retour_vert_bord',
                      Retour_vert_bord(),
                      transitions={'succeeded':'Turn_for_match','aborted':'problem'})

            smach.StateMachine.add('Turn_for_match',
                      Turn_for_match(),
                      transitions={'succeeded':'WaitForMatch','aborted':'problem'})


            smach.StateMachine.add('WaitForMatch', WaitForMatch(),
                                   transitions={'start':'gogogo'})
    
            

class Recal_hor_bord(CyclicActionState):
    def createAction(self):
       self.backward(0.300)
 
 
class SetPos_hor_bord(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeTransitions(self):
        return 'done'
        
    def executeIn(self):
        self.setPosition(Inputs.getx(),Table.HWALL_Y-Robot.DIST_BACK,-pi/2.0)


class Retour_hor_bord(CyclicActionState):
    def createAction(self):
        if Data.color=='red':
            self.pointcap(Inputs.getx(),Table.HWALL_Y-0.180,-pi/2)
        else:
            self.pointcap(Inputs.getx(),Table.HWALL_Y-0.220,-pi/2)
     

class hor_to_vert(CyclicActionState):
    def createAction(self):
       capobj=AmbiCapRed(0,Data.color)
       self.cap(capobj.angle)


class Recal_vert_bord(CyclicActionState):
    def createAction(self): 
        if Data.color=='red':
             poseRecalVert=AmbiPoseRed(-Table.VWALL_X-0.250,Table.HWALL_Y-0.180,0,Data.color)
        else:
            poseRecalVert=AmbiPoseRed(-Table.VWALL_X-0.250,Table.HWALL_Y-0.220,0,Data.color)
        
        self.pointcap_reverse(poseRecalVert.x,poseRecalVert.y,poseRecalVert.theta)


class SetPos_vert_bord(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeTransitions(self):
        return 'done'
        
    def executeIn(self):
        posRecal=AmbiPoseRed(-Table.VWALL_X+Robot.DIST_BACK,Inputs.gety(),0,Data.color)
        self.setPosition(posRecal.x,Inputs.gety(),posRecal.theta)


class Retour_vert_bord(CyclicActionState):
    def createAction(self):
        if Data.color=='red':
            poseStart=AmbiPoseRed(-Table.VWALL_X+Robot.DIST_BACK+0.030,Table.HWALL_Y-0.180,0,Data.color)
        else:
            poseStart=AmbiPoseRed(-Table.VWALL_X+Robot.DIST_BACK+0.030,Table.HWALL_Y-0.220,0,Data.color)
            
        self.pointcap(poseStart.x,poseStart.y,poseStart.theta)
       
       
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
        
