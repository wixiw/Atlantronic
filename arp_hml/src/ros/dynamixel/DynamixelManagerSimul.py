#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_hml')
import rospy
from std_msgs.msg import Float64
from arp_hml.msg import *
from dynamixel_msgs.msg import JointState
import actionlib

###
# This node is used to control the 4 dynamixel of the front specific 2012 claw. 
# It aims at synchronizing datas and measure and monitoring blocking situations
# It is an action server
#######
class DynamixelManager():
    
    def initRosItf(self):
        rospy.init_node('DynamixelManager')
        self.server = actionlib.SimpleActionServer('move_claw', ClawOrderAction, self.actionServerCb, False)
        
    
    def __init__(self):
        rospy.loginfo("DynamixelManager !!!SIMUL !!!: initializating ...")
        self.initRosItf()
        
        self.goalReached = False
        self.shutdown = False
        
    #a appeler des que l'objet est construit pour commencer a travailler. Appel bloquant
    def start(self):
        self.server.start()
        rospy.loginfo("DynamixelManager : started")
        while not rospy.is_shutdown():
            self.goalReached = True
            rospy.sleep(0.1)
            
        #quand on s'arrete on leve un flag pour quitter la callback action server
        rospy.loginfo('DynamixelManager main loop exited, waiting action server...')
        self.shutdown = True

    #callback appelee par l'action server lorsqu'un ordre est recu
    def actionServerCb(self, goal):
        rospy.loginfo("DynamixelManager : new order finger : (%s %s) claw : (%s %s)", goal.left_finger, goal.right_finger, goal.left_claw, goal.right_claw)
        self.goalReached = False
        result = ClawOrderResult(success=False)
        
        while not self.goalReached and not self.shutdown:
            if self.server.is_preempt_requested():
                rospy.loginfo('DynamixelManager Preempted')
                result.success = False
                self.server.set_preempted(result)
                return
            rospy.sleep(0.1)
  
        #on est sorti de la boucle parce qu'on a ferme le noeud
        if self.shutdown:
            self.server.set_aborted()
            rospy.loginfo('DynamixelManager action server exited')
            return
  
        #on a reussi
        result.success = True
        self.server.set_succeeded(result)
        rospy.loginfo('DynamixelManager action finished')
        return

        
if __name__ == '__main__':
    try:
        manager = DynamixelManager()
        manager.start()
    except rospy.ROSInterruptException: pass