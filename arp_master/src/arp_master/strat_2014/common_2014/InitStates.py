#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

print "-----arrivee dans inittstaes"


from arp_master import *

from Table2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *

#from arp_master.commonStates.Strat_Initialisation import *
#from arp_master.commonStates.Strat_RecalOnBorder import *
#from arp_master.commonStates.SetPosition import *

####################################################################################################################
print "-----DEFINITION StartSequence2014"
#for name in dir():
#            print ">>>>> " , name
            
class StartSequence2014(smach.StateMachine):
    def __init__(self,x,y,theta):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        
        for name in dir():
            print name
        
        with self:
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(1.350,0.500,0),
                      transitions={'succeeded':'WaitForStartUnPlug', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForStartUnPlug',
                      WaitForStartUnplug(),
                      transitions={'startunplug':'RecalX', 'timeout':'problem'})

            smach.StateMachine.add('RecalX',
                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
                      transitions={'recaled':'EscapeRecalX', 'non-recaled':'problem','problem':'problem'})
            
            smach.StateMachine.add('EscapeRecalX',
                      AmbiOpenLoopOrder(-0.1,0.0,0,0.5),
                      transitions={'succeeded':'PrepareRecalY', 'timeout':'PrepareRecalY'})
            
            smach.StateMachine.add('PrepareRecalY',
                      AmbiOmniDirectOrder2(Pose2D(0.900,0.500,pi/2), vmax = 0.3),
                      transitions={'succeeded':'RecalY', 'timeout':'problem'})
            
            smach.StateMachine.add('RecalY',
                      AmbiRecalOnBorderYellow("FRUITBASKET",Data.color),
                      transitions={'recaled':'EscapeRecalY', 'non-recaled':'problem','problem':'problem'})   
            
            smach.StateMachine.add('EscapeRecalY',
                      AmbiOpenLoopOrder(-0.1,0.0,0,0.5),
                      transitions={'succeeded':'ShowReady', 'timeout':'ShowReady'})
            
            smach.StateMachine.add('ShowReady',
                      AmbiOmniDirectOrder2(Pose2D(1.100,0.350,-pi/2), vmax = 0.3),
                      transitions={'succeeded':'WaitForLoc2', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForLoc2',
                      WaiterState(2.0),
                      transitions={'timeout':'GoHome'})
            
            smach.StateMachine.add('GoHome',
                      AmbiOmniDirectOrder2(Pose2D(x,y,theta), vmax = 0.3),
                      transitions={'succeeded':'WaitForStart', 'timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForStart', 
                       WaitForStart(),
                       transitions={'start':'WaitForMatch','timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'WaitForMatch'})


####################################################################################################################
