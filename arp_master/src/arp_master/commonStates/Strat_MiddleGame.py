#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.util import *
from arp_master.fsmFramework import *
from EndMatchPreempter import *
from Waiting import *

#
# @param : ActionSelector (a reference the year-dependent ActionSelector) 
class MiddleGame(PreemptiveStateMachine):
    def __init__(self, actionSelector):
        #As in any StateMachine that works for some time, an interruption is mapped to go to the EndGame State.
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame', 'motionBlocked'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddleGame', })
            
            #Select next Action and go to its entry point the mapping the the next Action state 
            #is delegated to next state TransitionToCurrentAction so that we don't mix movement transitions and Actions transitions
            PreemptiveStateMachine.add('GotoNextActionEntryPoint',
                      GotoNextAction(actionSelector),
                      transitions={'succeeded':'TransitToCurrentAction', 'timeout':'motionBlocked', 'nothingToDo':'endMiddleGame'})
            self.setInitialState('GotoNextActionEntryPoint')
    
            #this state is requiered to map the success transition of the move on the next Action State
            state = TransitToCurrentAction(actionSelector)
            PreemptiveStateMachine.add('TransitToCurrentAction',
                      state,
                      transitions = state.actionTransitionMap )
            
            #The list of Action states to be registered are automatically generated
            for actionDef in actionSelector.actionList:
                PreemptiveStateMachine.add(actionDef.actionName,
                                           actionDef.state,
                                           transitions={'succeeded':'SetActionSucceeded', 
                                                        'failed':'SetActionFailed',
                                                        'almostEndGame':'endMiddleGame'})
                
            #Update the action list to remember that current action has been done successfully                
            PreemptiveStateMachine.add('SetActionSucceeded',
                      SetActionResult('Done', actionSelector),
                      transitions={'done':'GotoNextActionEntryPoint'})
                            
            #Update the action list to remember that current action has failed                
            PreemptiveStateMachine.add('SetActionFailed',
                      SetActionResult('Failed', actionSelector),
                      transitions={'done':'GotoNextActionEntryPoint'})
     
         
      
        
#This state is asking to an ActionSelector the next Action to do and goes to its entry point
#it only moves to the target so the 2 outputs are 'succeeded' and 'timeout'. 
#if you want to select the next state you have to link with TransitToCurrentAction 
#on success in the parent state machine
# @param ActionSelector (the choice of the next action is really year dependent 
#                        so we delegate this to an external ActionSelector) 
class GotoNextAction(MotionState):
    def __init__(self, actionSelector):
        MotionState.__init__(self, outcomes = ['nothingToDo'])
        
        #a persistant memory of the parameter for the createAction function
        self.actionSelector = actionSelector
    
    def execute(self, userdata):
        if self.actionSelector.selectNextAction() is None:
            return 'nothingToDo'
        else:
           return MotionState.execute(self, userdata) 
        
    def createAction(self):
        self.pose = self.actionSelector.getCurrentEntryYellowPose2D()
        self.omnidirect2(self.pose.x, self.pose.y, self.pose.theta, -1)
        
        
        
      
#This State cane be used to generate a transition for each existing Action in Robot.actionList     
#it usually follows a GotoNextAction State 
# @param : ActionSelector (a reference the year-dependent ActionSelector) 
class TransitToCurrentAction(smach.State):
    def __init__(self, actionSelector):
        #a persistant memory of the parameter for the createAction function
        self.actionSelector = actionSelector
        
        #Auto-generated list of transitions related to the list possible Actions
        #this is an helper function for the MiddleGame state machine
        #it is of the form : ['action1','action2','action3']
        self.actionTransitionList = list()
        self.generateActionTransitionList()
        
        #Auto-generated dictionnary of transtion-to-State mappings
        #this is an helper function for the MiddleGame state machine
        #it is of the form : {'action1':'Action1', 'action2':'Action2', 'action3':'Action3'}
        self.actionTransitionMap = dict()
        self.generateTransitionToActionMap()
        
        smach.State.__init__(self, outcomes=self.actionTransitionList)
        
        
    def execute(self, userdata):   
        return self.actionSelector.getCurrentActionDef().transitionName

    def generateActionTransitionList(self):    
        for actionDef in self.actionSelector.actionList:
            self.actionTransitionList.append(actionDef.transitionName)
        self.actionTransitionList.append('nearlyEndMatch')

    def generateTransitionToActionMap(self):    
        for actionDef in self.actionSelector.actionList:
            self.actionTransitionMap[actionDef.transitionName] = actionDef.actionName
        self.actionTransitionMap['nearlyEndMatch'] = 'endMiddleGame'
  
  
   
   
#This State is updating the strategic memory of action result status.
# @param : String (the action result, see RobotVierge.py=>ActionDefinition for details)                   
class SetActionResult(smach.State):
    def __init__(self, actionResult, actionSelector):
        #a persistant memories of input parameters for the execute function
        self.actionResult = actionResult
        self.actionSelector = actionSelector
        
        smach.State.__init__(self, outcomes=['done'])
    
    def execute(self, userdata):
        self.actionSelector.setCurrentActionResult(self.actionResult)
        self.actionSelector.__repr__()
        return 'done'
    
    
        
        
             