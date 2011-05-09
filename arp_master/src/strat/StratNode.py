#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
# library for the state machine
import smach
import smach_ros
import smach_msgs
# import the definition of the messages
from arp_core.msg import Obstacle
from arp_core.msg import StartColor
from arp_core.msg import Start
#import the other strat modules    
import CyclicState
import Strat_Initialisation

###########################  TEMPORAL BEHAVIOR

def StratNode():
    global inputList,stateMachineRate
    
    rospy.init_node('StratNode')
    stateMachineRate =rospy.Rate(1)
    
    init()
    
    while not rospy.is_shutdown():
        for input in inputList:
            input.update()
        mainloop()
        stateMachineRate.sleep()
        
############################# INITIALISATION
def init():
    
    #creation and linking of inputs
    global inputList,obstacle,color,start    
    inputList=[]
    obstacleInput= Input("obstacle", Obstacle)
    obstacle=obstacleInput.data.detected
    colorInput=Input("color", StartColor)
    color=colorInput.data.color
    startInput=Input("start", Start)
    start=startInput.data.go
    #x = Input()
    #y = Input()
    #theta = Input()

    #creation of statemachine
    sm=MainStateMachine()
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()

    #welcome message
    rospy.loginfo("******************************************************")
    rospy.loginfo("Welcome to Advanced Robotics Platform. I am StratNode.")
    rospy.loginfo("Choose color with button")
    rospy.loginfo("Then plug start")
    rospy.loginfo("Wait for initialisation sequence")
    rospy.loginfo("And unplug start")
    rospy.loginfo("******************************************************")

    sm.execute()
    
    
############################# MAIN LOOP
def mainloop():
    global obstacle
    rospy.loginfo("obstacle: %i"%obstacle.data.detected)
    rospy.loginfo("callback appelee: %i fois"%obstacle.ncall)
    
    
############################## STATE MACHINE
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'end'})


########################### INPUT CLASS
# this is used to have a buffer on inputs, so that they are not changing during the mainloop
class Input():
    def __init__(self,topicName,msgType):
        global inputList
        self.data=msgType()
        self.data_lastvalue=msgType()
        self.ncall=0
        self.ncall_lastvalue=0
        inputList.append(self)
        rospy.Subscriber(topicName,msgType,self.updateCallBack)
    def update(self):
        self.data=self.data_lastvalue
        self.ncall=self.ncall_lastvalue
        self.ncall_lastvalue=0
    def updateCallBack(self, data):
        self.data_lastvalue=data
        self.ncall_lastvalue+=1    
   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        StratNode()
    except rospy.ROSInterruptException: pass