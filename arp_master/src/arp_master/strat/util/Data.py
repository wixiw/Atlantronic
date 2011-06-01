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
    pionBordObjectif=None

    liste_cases2_faites=[]
    
    listReplayOrders=[]
    
    timeObstacle=0
    timeRearObstacle=0
    
    #autorisation de relocalisation
    allowRelocate=False
    
    #type d'evitement d'obstacle
    #ce sont les etats qui le changent pour que la preemption reagisse differemment
    obstacleAvoidType='None'
    rearObstacleAvoidType='None'
    
    #utilise pour bloquer a chaque mouvement
    lastStart=1