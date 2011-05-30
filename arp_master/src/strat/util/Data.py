#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

class Data:

    #proprietes
    stateMachineRate=None
    
    #variables de la machine d'etat
    color='NONE'
    adv_color='NONE'
    going_back=False

    liste_cases2_faites=[]
    
    time_obstacle=0