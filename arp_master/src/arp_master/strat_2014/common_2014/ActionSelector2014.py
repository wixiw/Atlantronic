#! /usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
import os

from arp_master import *
from arp_master.util import *
from arp_master.strat_2014.localStrategicActions_2014 import *

actionListTanguy2014 = [
    ActionDefinition('StickFrescosState', StickFrescosState()), 
    ActionDefinition('DoubleTargetShootState', DoubleTargetShootState()),

    ActionDefinition('PickSelfFireTop', PickFireOnTable(Table2014.P_SELF_FIRE_TOP)),
    ActionDefinition('PickSelfFireMid', PickFireOnTable(Table2014.P_SELF_FIRE_MID)),
    ActionDefinition('PickSelfFireBot', PickFireOnTable(Table2014.P_SELF_FIRE_BOT)),              
    ActionDefinition('PickOppFireBop', PickFireOnTable(Table2014.P_OPP_FIRE_BOT)),                
    ActionDefinition('PickOppFireMid', PickFireOnTable(Table2014.P_OPP_FIRE_MID)),                
    ActionDefinition('PickOppFireTop', PickFireOnTable(Table2014.P_OPP_FIRE_TOP))
    #Warning: remettre la virgule quand on decommente
    
#    ActionDefinition('PickSelfTorchMid', PickFireOnBorder(Table2014.P_SELF_TORCH_MID)),
#    ActionDefinition('PickSelfTorchBot', PickFireOnBorder(Table2014.P_SELF_TORCH_BOT)),    
#    ActionDefinition('PickOppTorchBot', PickFireOnBorder(Table2014.P_OPP_TORCH_BOT)),
#    ActionDefinition('PickOppTorchMid', PickFireOnBorder(Table2014.P_OPP_TORCH_MID)),
                
          
]

class ActionSelector2014(ActionSelector):
    def __init__(self):
        ActionSelector.__init__(self,actionListTanguy2014)

        
#TODO : refine the select next action function
    