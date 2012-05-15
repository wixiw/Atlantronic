#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.TableVierge import *

import rospy
import smach

from arp_master.util.Data import *

#il faut voir ca comme un namespace
class Table2012(TableVierge):
        #
        # Points d'interet
        #     les points sont defini cote rouge comme toujours
        
        #bouteilles
        P_BOTTLE_CLOSE      = Point(0.860,-0.900)
        P_BOTTLE_FAR        = Point(-0.383,-0.900)
        #pieces
        P_COIN_TOP_CLOSE    = Point(0.500,0.500)
        P_COIN_TOP_FAR      = Point(-0.500,0.500)
        P_COIN_BOT_CLOSE    = Point(0.1050,-0.700)
        P_COIN_BOT_FAR      = Point(-0.1050,-0.700)
        #lingots
        P_GOLDBAR_CENTER    = Point(0,-0.353)
        #P_GOLDBAR_FREE_CLOSE
        #P_GOLDBAR_FREE_FAR
        #P_GOLDBAR_TOTEM_TOP_CLOSE
        #P_GOLDBAR_TOTEM_BOT_CLOSE
        #P_GOLDBAR_TOTEM_TOP_FAR
        #P_GOLDBAR_TOTEM_BOT_FAR
        
        #balises
        P_RED_BEACON_UP    = Point(1.565, 1.040)
        P_RED_BEACON_DOWN    = Point(1.565,-1.040)
        P_RED_BEACON_LEFT    = Point(-1.560, 0.)

        #poitns d'engagement des totems pour racler
        #P_ENGAGE_TOTEM_TOP_CLOSE
        #P_ENGAGE_TOTEM_BOT_CLOSE
        #P_ENGAGE_TOTEM_TOP_FAR
        #P_ENGAGE_TOTEM_BOT_FAR
        
        # 
        # Memoire des actions
        #
        closeBottlePushed = False
        farBottlePushed = False
        
        topCloseCoinInPosition = True
        topFarCoinInPosition = True
        botCloseCoinInPosition = True
        botFarCoinInPosition = True
        middleCoinsInPosition = True
        
        closeFreeGoldbarInPosition = True
        farFreeGoldbarInPosition = True
        middleGoldbarInPosition = True
        
        topCloseTotemFull = True
        topFarTotemFull = True
        botCloseTotemFull = True
        botFarTotemFull = True
        
        @staticmethod
        def printStratInfo():
            rospy.loginfo("*********************************************")
            rospy.loginfo("StratInfo :")
            rospy.loginfo("closeBottlePushed = %s",Table2012.closeBottlePushed)
            rospy.loginfo("farBottlePushed = %s",Table2012.farBottlePushed)
            
            rospy.loginfo("topCloseCoinInPosition = %s",Table2012.topCloseCoinInPosition)
            rospy.loginfo("topFarCoinInPosition = %s",Table2012.topFarCoinInPosition)
            rospy.loginfo("botCloseCoinInPosition = %s",Table2012.botCloseCoinInPosition)
            rospy.loginfo("botFarCoinInPosition = %s",Table2012.botFarCoinInPosition)
            rospy.loginfo("middleCoinsInPosition = %s",Table2012.middleCoinsInPosition)
            
            rospy.loginfo("closeFreeGoldbarInPosition = %s",Table2012.closeFreeGoldbarInPosition)
            rospy.loginfo("farFreeGoldbarInPosition = %s",Table2012.farFreeGoldbarInPosition)
            rospy.loginfo("middleGoldbarInPosition = %s",Table2012.middleGoldbarInPosition)
            
            rospy.loginfo("topCloseTotemFull = %s",Table2012.topCloseTotemFull)
            rospy.loginfo("topFarTotemFull = %s",Table2012.topFarTotemFull)
            rospy.loginfo("botCloseTotemFull = %s",Table2012.botCloseTotemFull)
            rospy.loginfo("botFarTotemFull = %s",Table2012.botFarTotemFull)
            
            rospy.loginfo("")
            rospy.loginfo("*********************************************")
            
            
# Use this state to define a variable 
class SetStratInfoState(smach.State):
    def __init__(self, strat_info,value):
        self.strat_info = strat_info
        self.value = value
        smach.State.__init__(self,['ok'])
    def execute(self,userdata):
        
        rospy.loginfo("SetStratInfoState : %s is set to %s (time=%s)", 
                      self.strat_info, self.value, (rospy.get_rostime()-Data.start_time).to_sec())
        
        if self.strat_info == "closeBottlePushed":
            Table2012.closeBottlePushed = self.value
        elif self.strat_info == "farBottlePushed":
            Table2012.farBottlePushed = self.value
            
        elif self.strat_info == "topCloseCoinInPosition":
            Table2012.topCloseCoinInPosition = self.value            
        elif self.strat_info == "topFarCoinInPosition":
            Table2012.topFarCoinInPosition = self.value            
        elif self.strat_info == "botCloseCoinInPosition":
            Table2012.botCloseCoinInPosition = self.value
        elif self.strat_info == "botFarCoinInPosition":
            Table2012.botFarCoinInPosition = self.value            
        elif self.strat_info == "middleCoinsInPosition":
            Table2012.middleCoinsInPosition = self.value   
            
        elif self.strat_info == "closeFreeGoldbarInPosition":
            Table2012.closeFreeGoldbarInPosition = self.value
        elif self.strat_info == "farFreeGoldbarInPosition":
            Table2012.farFreeGoldbarInPosition = self.value            
        elif self.strat_info == "middleGoldbarInPosition":
            Table2012.middleGoldbarInPosition = self.value          
             
        elif self.strat_info == "topCloseTotemFull":
            Table2012.topCloseTotemFull = self.value
        elif self.strat_info == "topFarTotemFull":
            Table2012.topFarTotemFull = self.value            
        elif self.strat_info == "botCloseTotemFull":
            Table2012.botCloseTotemFull = self.value            
        elif self.strat_info == "botFarTotemFull":
            Table2012.botFarTotemFull = self.value
        else:
            rospy.logerr("SetStratInfoState : default case : strat info (%s)  not known !!", self.strat_info)
                           
        return 'ok'
    
    
class TotemPose():
    def __init__(self, x,y,h, table_half):
        if table_half == "top_close":
            self.x = x
            self.y = y
            self.h = normalizeAngle(h)
        elif table_half == "bot_close":
            self.x = x
            self.y = -y
            self.h = -normalizeAngle(h)
        elif table_half == "top_far":
            self.x = -x
            self.y = y
            self.h = normalizeAngle(pi-h)
        elif table_half == "bot_far":
            self.x = -x
            self.y = -y
            self.h = -normalizeAngle(pi-h) 
        else:
            self.x = 0
            self.y = 0
            self.h = 0
            spy.logerr("TotemPose : default case, unknown table_half")