#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.TableVierge import *

class Table2012(TableVierge):
    def __init__(self):
        TableVierge.__init__(self)
        rospy.loginfo("Init Table 2012 datas ...")
        #
        # Points d'interet
        #     les points sont defini cote rouge comme toujours
        
        #bouteilles
        self.P_BOTTLE_CLOSE      = Point(860,-900)
        self.P_BOTTLE_FAR        = Point(-383,-900)
        #pieces
        self.P_COIN_TOP_CLOSE    = Point(500,500)
        self.P_COIN_TOP_FAR      = Point(-500,500)
        self.P_COIN_BOT_CLOSE    = Point(1050,-700)
        self.P_COIN_BOT_FAR      = Point(-1050,-700)
        #lingots
        self.P_GOLDBAR_CENTER    = Point(0,-353)
        #self.P_GOLDBAR_FREE_CLOSE
        #self.P_GOLDBAR_FREE_FAR
        #self.P_GOLDBAR_TOTEM_TOP_CLOSE
        #self.P_GOLDBAR_TOTEM_BOT_CLOSE
        #self.P_GOLDBAR_TOTEM_TOP_FAR
        #self.P_GOLDBAR_TOTEM_BOT_FAR
        
        #poitns d'engagement des totems pour racler
        #self.P_ENGAGE_TOTEM_TOP_CLOSE
        #self.P_ENGAGE_TOTEM_BOT_CLOSE
        #self.P_ENGAGE_TOTEM_TOP_FAR
        #self.P_ENGAGE_TOTEM_BOT_FAR
        
        # 
        # Memoire des actions
        #
        self.closeBottlePushed = False
        self.farBottlePushed = False
        
        self.topCloseCoinInPosition = True
        self.topFarCoinInPosition = True
        self.botCloseCoinInPosition = True
        self.botFarCoinInPosition = True
        self.MiddleCoinsInPosition = True
        
        self.closeFreeGoldbarInPosition = True
        self.farFreeGoldbarInPosition = True
        self.middleGoldbarInPosition = True