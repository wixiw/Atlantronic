#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.TableVierge import *

#il faut voir ca comme un namespace
class Table2012(TableVierge):
        #
        # Points d'interet
        #     les points sont defini cote rouge comme toujours
        
        #bouteilles
        P_BOTTLE_CLOSE      = Point(860,-900)
        P_BOTTLE_FAR        = Point(-383,-900)
        #pieces
        P_COIN_TOP_CLOSE    = Point(500,500)
        P_COIN_TOP_FAR      = Point(-500,500)
        P_COIN_BOT_CLOSE    = Point(1050,-700)
        P_COIN_BOT_FAR      = Point(-1050,-700)
        #lingots
        P_GOLDBAR_CENTER    = Point(0,-353)
        #P_GOLDBAR_FREE_CLOSE
        #P_GOLDBAR_FREE_FAR
        #P_GOLDBAR_TOTEM_TOP_CLOSE
        #P_GOLDBAR_TOTEM_BOT_CLOSE
        #P_GOLDBAR_TOTEM_TOP_FAR
        #P_GOLDBAR_TOTEM_BOT_FAR
        
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
        MiddleCoinsInPosition = True
        
        closeFreeGoldbarInPosition = True
        farFreeGoldbarInPosition = True
        middleGoldbarInPosition = True
        
        topCloseTotemFull = True
        topFarTotemFull = True
        botCloseTotemFull = True
        botFarTotemFull = True
        
        def __init__(self):
            TableVierge.__init__(self)
            rospy.loginfo("Init Table 2012 datas ...")