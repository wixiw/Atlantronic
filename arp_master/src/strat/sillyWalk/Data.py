#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

class Data:
    #proprietes
    stateMachineRate=None
    
    #variables de la machine d'etat
    color='NONE'

