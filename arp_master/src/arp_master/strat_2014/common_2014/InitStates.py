#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from Table2014 import *

####################################################################################################################


class StartSequence2014(smach.StateMachine):
    def __init__(self,x,y,theta):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('FindSteeringZeros',
                      FindSteeringZeros(), 
                      transitions={'succeeded':'SetInitialPosition', 'problem':'problem'})
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(1.350,0.500,0),
                      transitions={'succeeded':'WaitForStartUnPlug', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForStartUnPlug',
                      WaitForStartUnplug(),
                      transitions={'startunplug':'RecalX', 'timeout':'problem'})
            
            #todo faire un etat recalage
            smach.StateMachine.add('RecalX',
                      AmbiOpenLoopOrder(0.1,0.0,0,2.0),
                      transitions={'succeeded':'SetRecalXPosition', 'timeout':'SetRecalXPosition'})
            
            #TODO : mettre la valeur de la distance 1500-face avant
            smach.StateMachine.add('SetRecalXPosition',
                      SetInitialPosition(1.450,0.500,0),
                      transitions={'succeeded':'Debug1', 'timeout':'problem'})
            
            smach.StateMachine.add('Debug1', 
                       WaitForStart(),
                       transitions={'start':'Debug2','timeout':'WaitForStart'})
            smach.StateMachine.add('Debug2',
                      WaitForStartUnplug(),
                      transitions={'startunplug':'EscapeRecalX', 'timeout':'problem'})
                        
            smach.StateMachine.add('EscapeRecalX',
                      AmbiOpenLoopOrder(-0.1,0.0,0,0.5),
                      transitions={'succeeded':'PrepareRecalY', 'timeout':'PrepareRecalY'})
            
            smach.StateMachine.add('PrepareRecalY',
                      AmbiOmniDirectOrder2(0.900,0.500,pi/2, vmax = 0.3),
                      transitions={'succeeded':'RecalY', 'timeout':'problem'})
            
            #todo faire un etat recalage
            smach.StateMachine.add('RecalY',
                      AmbiOpenLoopOrder(0.1,0.0,0,2.0),
                      transitions={'succeeded':'SetRecalYPosition', 'timeout':'SetRecalYPosition'})
            
            #TODO : mettre la valeur de la distance 1500-face avant
            smach.StateMachine.add('SetRecalYPosition',
                      SetInitialPosition(0.900,0.650,pi/2),
                      transitions={'succeeded':'EscapeRecalY', 'timeout':'problem'})
            
            smach.StateMachine.add('EscapeRecalY',
                      AmbiOpenLoopOrder(-0.1,0.0,0,0.5),
                      transitions={'succeeded':'ShowReady', 'timeout':'ShowReady'})
            
            smach.StateMachine.add('ShowReady',
                      AmbiOmniDirectOrder2(1.100,0.350,-pi/2, vmax = 0.3),
                      transitions={'succeeded':'WaitForLoc2', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForLoc2',
                      WaiterState(2.0),
                      transitions={'timeout':'GoHome'})
            
            smach.StateMachine.add('GoHome',
                      AmbiOmniDirectOrder2(x,y,theta, vmax = 0.3),
                      transitions={'succeeded':'WaitForStart', 'timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForStart', 
                       WaitForStart(),
                       transitions={'start':'WaitForMatch','timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'WaitForMatch'})


####################################################################################################################
