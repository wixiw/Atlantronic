#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy


class Data:
    #temps de cycle de la machine d'etat
    stateMachineRate=None
    
    #couleur de jeu
    color='NONE'
    adv_color='NONE'

    #derniers ordres. utilise pour le replay
    listReplayOrders=[]
    
    #temps lies a la detection adversaire
    timeObstacle=0
    timeRearObstacle=0
    timeObstacleInAction=0
    timeRearObstacleInAction=0
    
    #type d'evitement d'obstacle
    #ce sont les etats qui le changent pour que la preemption reagisse differemment
    obstacleAvoidType='None'
    rearObstacleAvoidType='None'

    
    
