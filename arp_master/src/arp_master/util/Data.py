#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

from arp_master import *
from arp_master.util.RobotVierge import * 
from arp_master.util.TableVierge import * 

#il faut voir ca comme un namespace
class Data:
    #temps de cycle de la machine d'etat
    stateMachineRate=None
    
    #couleur de jeu
    color='NONE'
    adv_color='NONE'

    #date de debut de match :
    start_time = 0

    #temps lies a la detection adversaire
    timeObstacle=0
    timeRearObstacle=0
    timeObstacleInAction=0
    timeRearObstacleInAction=0
    
    #type d'evitement d'obstacle
    #ce sont les etats qui le changent pour que la preemption reagisse differemment
    obstacleAvoidType='None'
    rearObstacleAvoidType='None'
    
    #stockge des donnees de deplacement pour verifier si on va peter le cul de l'adversaire sur notre trajectoire
    #est-ce une translation ?
    isMotionTranslation = False
    motionTarget=Point(0,0)
    

    
    
