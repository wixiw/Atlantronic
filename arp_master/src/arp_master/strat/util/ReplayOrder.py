#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

from Inputs import Inputs
from Data import Data

from arp_ods.msg import OrderGoal

class ReplayOrder:
    def __init__(self,x,y,theta,move_type,reverse,passe):
        #je note l'heure de passage
        self.time=rospy.get_rostime().secs
        
        self.goal=OrderGoal()
        self.goal.x_des=x
        self.goal.y_des=y
        self.goal.theta_des=theta
        self.goal.move_type=move_type
        self.goal.reverse=reverse
        self.goal.passe=passe
        
        #je cree l'ordre qui me permettra de revenir en arriere au debut...
        self.reversegoal=OrderGoal()
        self.reversegoal.x_des=Inputs.getx()
        self.reversegoal.y_des=Inputs.gety()
        self.reversegoal.theta_des=Inputs.gettheta()
        self.reversegoal.reverse=not reverse
        self.reversegoal.move_type=move_type
        self.reversegoal.passe=False