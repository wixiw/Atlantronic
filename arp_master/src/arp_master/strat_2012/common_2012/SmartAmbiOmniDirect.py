#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *
from Robot2012 import *
from Table2012 import *

# A utiliser pour aller quelque part sans se payer le decor
# Soit on veut aller dans le quart oppose et il faut 3 points de passage
# Soit on veut aller dans le quart d'a cote et il faut 2 points de passage
# Soit on reste dans notre quart et on a 1 point de passage
# Les points de passges sont les "centres" des quarts de table avec un cap qui fait viser la face avant vers le "centre"

            
class SmartAmbiOmniDirectOrder(PreemptiveStateMachine):
    def __init__(self,x,y,theta, vmax=-1.0):
        PreemptiveStateMachine.__init__(self,outcomes=['succeeded','timeout'])
        with self:    
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'timeout'})
            
            PreemptiveStateMachine.add('GoInOurHalf',
                      GoInOurHalf(), 
                      transitions={'succeeded':'ChooseState', 'timeout':'timeout'})
            self.setInitialState('GoInOurHalf')
            
            
            PreemptiveStateMachine.add('ChooseState',
                      ChooseState(x,y,theta), 
                      transitions={'inHalfTable':'GoToTarget', 'inNearHalfTable':'GoInTargetHalf', 'inDiagHalfTable':'GoDiagHalf', 
                                   'timeout':'timeout'})
            
            
            PreemptiveStateMachine.add('GoDiagHalf',
                      GoNextHalf(), 
                      transitions={'succeeded':'GoInTargetHalf', 'timeout':'timeout'})


            PreemptiveStateMachine.add('GoInTargetHalf',
                      GoInTargetHalf(x,y,theta),
                      transitions={'succeeded':'GoToTarget', 'timeout':'timeout'})
            
            
            PreemptiveStateMachine.add('GoToTarget',
                      AmbiOmniDirectOrder(x,y,theta), 
                      transitions={'succeeded':'succeeded', 'timeout':'timeout'})


class ChooseState(CyclicState):
    def __init__(self,x,y,theta):
        CyclicState.__init__(self, outcomes=['inHalfTable','inNearHalfTable','inDiagHalfTable'])
        self.x = x
        self.y = y
        self.theta = theta

    def executeTransitions(self):
        robot_table_half = Table2012.getTableHalf(Inputs.getx(),Inputs.gety(),Data.color)
        target = AmbiPoseRed(self.x,self.y,0,Data.color)
        target_table_half = Table2012.getTableHalf(target.x,target.y,Data.color)
        if  robot_table_half == target_table_half: 
            return 'inHalfTable'
        elif robot_table_half == Table2012.getOppositeHalf(target_table_half):
            return 'inDiagHalfTable'
        else:
            return 'inNearHalfTable'

class GoInHalf(MotionState):
        def __init__(self, table_half):
            MotionState.__init__(self)
            self.table_half = table_half

        def createAction(self):
            if  self.table_half == "closeTop":
                target = AmbiPoseRed(0.800,0.700,-3*pi/4,Data.color)
            elif  self.table_half == "closeBot":
                target = AmbiPoseRed(0.800,-0.700,3*pi/4,Data.color)
            elif  self.table_half == "farBot":
                target = AmbiPoseRed(-0.800,-0.700,pi/4,Data.color)
            elif  self.table_half == "farTop":
                target = AmbiPoseRed(-0.800,0.700,-pi/4,Data.color)
            else:
                rospy.loginfo("Failed to GoInHalf %s", self.table_half)
                target = AmbiPoseRed(0.800,0.700,-3*pi/4,Data.color)
                
            #rospy.loginfo("***** going in half %s (%s,%s,%s)", self.table_half,target.x, target.y, target.theta)
            self.omnidirect(target.x, target.y, target.theta)


class GoInOurHalf(GoInHalf):
        def __init__(self):
            GoInHalf.__init__(self,"dummy")

        def createAction(self):
            self.table_half = Table2012.getTableHalf(Inputs.getx(),Inputs.gety(),Data.color)
            GoInHalf.createAction(self)
            
class GoInTargetHalf(GoInHalf):
        def __init__(self,x,y,theta):
            GoInHalf.__init__(self,"dummy")
            self.x = x
            self.y = y 
            self.theta = theta

        def createAction(self):
            target = AmbiPoseRed(self.x,self.y,0,Data.color)
            self.table_half = Table2012.getTableHalf(target.x,target.y,Data.color)
            GoInHalf.createAction(self)
            
class GoNextHalf(GoInHalf):
        def __init__(self):
            GoInHalf.__init__(self,"dummy")

        def createAction(self):
            robot_current_half = Table2012.getTableHalf(Inputs.getx(),Inputs.gety(),Data.color)
            self.table_half = Table2012.getNextHalfAmbiHoraire(robot_current_half)
            GoInHalf.createAction(self)