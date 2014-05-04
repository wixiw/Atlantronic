#! /usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
import os

from arp_master import *
from arp_master.util import *


#TODO : update the list with your real Actions
actionListVierge = [
    ActionDefinition('Wait3', WaiterState(3.0)), 
    ActionDefinition('Wait1', WaiterState(1.0))             
]
 
 
class ActionSelectorV(ActionSelector):
    def __init__(self):
        ActionSelector.__init__(actionListVierge)

#TODO : refine the select next action function
    