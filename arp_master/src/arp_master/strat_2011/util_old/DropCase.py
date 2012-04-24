#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

from Table2011 import *


#classe qui sert a symboliser une case de depose: on lui rajoute une occupation 
class DropCase(Case):
    def __init__(self,i_hor,j_vert):
        self.i=i_hor
        self.j=j_vert
        self.status='VIRGIN'
    
    def getCase(self,color):
        return AmbiCaseRed(self.i,self.j,color)