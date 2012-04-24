#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

from Inputs import Inputs
from Data import Data

from arp_ods.msg import OrderGoal

class ReplayOrder:
    def __init__(self,x_cpoint,y_cpoint,theta_cpoint,x,y,theta,move_type,reverse,passe):
        #what time I came in
        self.time=rospy.get_rostime().secs
        
        # "goal" is where I want to go
        self.goal=OrderGoal()
        self.goal.x_des=x
        self.goal.y_des=y
        self.goal.theta_des=theta
        self.goal.x_des=x_cpoint
        self.goal.y_des=y_cpoint
        self.goal.theta_des=theta_cpoint
        self.goal.move_type=move_type
        self.goal.reverse=reverse
        self.goal.passe=passe
        
        # "reverse goal" is where I am now, and the reverse motion parameters that will allow me to come back to this point
        self.reversegoal=OrderGoal()
        #############################################TODO il faut enregistrer la position du point de controle ! pas le ref du robot
        self.reversegoal.x_des=Inputs.getx()
        self.reversegoal.y_des=Inputs.gety()
        self.reversegoal.theta_des=Inputs.gettheta()
        self.reversegoal.reverse=not reverse
        self.reversegoal.move_type=move_type
        self.reversegoal.passe=False