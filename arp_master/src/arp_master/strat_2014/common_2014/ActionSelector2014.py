#! /usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
import os

from arp_master import *
from arp_master.util import *
from FrescosStates import *
from ShootStates import *

actionListTanguy2014 = [
    ActionDefinition('StickFrescosState', StickFrescosState()), 
    ActionDefinition('DoubleTargetShootState', DoubleTargetShootState())             
]
 
 
class ActionSelector2014(ActionSelector):
    def __init__(self):
        ActionSelector.__init__(self,actionListTanguy2014)

        
#TODO : refine the select next action function
    