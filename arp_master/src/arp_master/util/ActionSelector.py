#! /usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
import os

#An ActionDefinition is used by the Middlegame state machine. It's a mapping between :
# _ an atomic strategic state class (named an "Action"), 
# _ its instance name
# _ its dedicated transition name in the MidleGameState
# _ the Action persistant state 
class ActionDefinition():
    def __init__(self, p_actionName, p_state):
        self.actionName = p_actionName                        #the name of the Action instance (can be "MyBelovedActionXXX")
        self.actionResult = 'NotDone'                       #can be : 'Done', 'NotDone', 'Failed'
        self.transitionName = self.lowerFirstLetter(p_actionName)  #transitions are always in lower case, so the generator is following the rule ;p
        self.state = p_state                                  #State instance such as "WaiterState(3.0)"

    #Standard toString() method in python        
    def __repr__(self):
        return "\n[%s\n\t| %s\n\t| %s]" % (self.actionName, self.transitionName, self.actionResult)
        
    def lowerFirstLetter(self,s):
       if not s: # Added to handle case where s == None
           return 
       else:
          return s[0].lower() + s[1:]


#This class is responsible of choosing the next Action for the robot
#it aims at being overrided in a year-dependent manner. Most of the time, 
#the selectNextAction is the only function to refine
# @param : list(ActionDefinition) (the list of Actions that the robot knows)
class ActionSelector:
    def __init__(self, actionList):
        #List of possible Action 
        self.actionList = actionList
        
        #Index of the current ActionDefiition in the actionList
        self.currentActionId = None
        
        self.__repr__()
        
    #This is a default implementation that a year dependent action selector should override
    #This implementation choose the next action in the list prefering 'NotDone' actions, then 'Failed' actions
    #it stops when there is only one Failed action and a list of Done actions.
    # @return : ActionDefinition (the next action definition that has been set to be the current one)
    #           may be empty if don't know what to choose
    def selectNextAction(self):
        result = None
        
        if self.doesActionListContainsResultOfType('NotDone') is True:
            self.currentActionId = self.findNextActionIdWithResult('NotDone')
            print "[ActionSelector] : A NotDone action is available : " + self.getCurrentActionDef().__repr__()
            result = self.getCurrentActionDef()
            
        elif self.doesActionListContainsResultOfType('Failed') is True:
            self.currentActionId = self.findNextActionIdWithResult('Failed')
            print "[ActionSelector] : A Failed action can be re-tried : " + self.getCurrentActionDef().__repr__()
            result = self.getCurrentActionDef()
            
        else:
            self.currentActionId = -1
            print "[ActionSelector] : I don't know what to do :("
            result = None
            
        print "[ActionSelector] : new ID is : " + str(self.currentActionId)
        return result
    
    #Every action shall provide an entry point. 
    #The "Global" strategy is responsible to move the robot between Action entry points
    #Actions represent the "Local" strategy.
    # @return : Pose2D (the entry point)
    def getCurrentEntryYellowPose2D(self):
        return self.getCurrentActionDef().state.getEntryYellowPose()
    
    #Retrieves the current ActionDefinition from the list of existing actions.
    # @return : ActionDefinition (the current action def handle, may be empty if the index is out of range)
    def getCurrentActionDef(self):
        if self.currentActionId is None:
            return None
        else:
            return self.actionList[self.currentActionId]
        

    #Retrieves the action result in the current ActionDefinition
    #See ActionDefinition class to know more about possible values
    # @return : String (the action result)
    def getCurrentActionResult(self):
        return self.getCurrentActionDef().actionResult
        
    #Browse the action list to find at least one actionResult of type="actionResult"
    # @param String (the action result description string => ActionDefinition class)
    # @return Boolean = True if at least one actionResult is equal to actionResult 
    def doesActionListContainsResultOfType(self, p_actionResult):
        for actionDef in self.actionList:
            if actionDef.actionResult == p_actionResult:
                return True
        #nothing found
        return False
    
    # Browse the action list and return the first ActionDef id in the action list 
    # whose result is p_actionResult. The browsing begins with self.currentActionId
    # @returns : Integer (the next action ID, may be -1 on error)
    def findNextActionIdWithResult(self, p_actionResult):
        if self.currentActionId is None:
            print "first time id=0"
            return 0
        
        id = self.currentActionId + 1
        if len(self.actionList) <= id:
            id = 0
            
        while id != self.currentActionId:
            print "self.actionList[" + str(id) + "] = " + self.actionList[id].actionResult + " tested against " + p_actionResult
            if self.actionList[id].actionResult == p_actionResult:
                print "found id=" + str(id)
                return id
            else:
                id += 1
                if len(self.actionList) <= id:
                    id = 0
                    print "id reseted"

        print "no id found"      
        return None
    
            
    #Defines the action result in the current ActionDefinition
    #See ActionDefinition class to know more about possible values
    # @param : String (the action result)
    def setCurrentActionResult(self, p_actionResult):
        self.getCurrentActionDef().actionResult = p_actionResult

    #Standard toString() method in python        
    def __repr__(self):
        print "[ActionSelector:]"
        print "|--------------------------------------------|"
        print "| Action List is :                           |"
        print "|--------------------------------------------|"
        print self.actionList
        print "|--------------------------------------------|"
        print "| Current Id " + str(self.currentActionId) 
        print "|--------------------------------------------|"
        print ""
        print ""
        
