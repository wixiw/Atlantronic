#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

from DropCase import *
    
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
    
    casePriority=[DropCase(3,3),DropCase(3,-1)]
    
    randomDropList=[DropCase(-3,5),DropCase(-5,3),DropCase(-5,-1),DropCase(-3,-3),DropCase(1,-3)]
    
    listStatusPionsBord=['NORMAL','NORMAL','NORMAL','NORMAL']
    
    timeObstacle=0
    timeRearObstacle=0
    timeObstacleInAction=0
    timeRearObstacleInAction=0
    
    #autorisation de relocalisation
    allowRelocate=False
    
    #type d'evitement d'obstacle
    #ce sont les etats qui le changent pour que la preemption reagisse differemment
    obstacleAvoidType='None'
    rearObstacleAvoidType='None'
    
    #utilise pour bloquer a chaque mouvement
    lastStart=1
    
    
