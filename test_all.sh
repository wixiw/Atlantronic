#!/bin/bash

VERT="\\033[1;32m" 
NORMAL="\\033[0;39m" 
ROUGE="\\033[1;31m" 
ROSE="\\033[1;35m" 
BLEU="\\033[1;34m" 
BLANC="\\033[0;02m" 
BLANCLAIR="\\033[1;08m" 
JAUNE="\\033[1;33m" 
CYAN="\\033[1;36m" 


#CORE
rosrun arp_core uTest_Core_Math.ard
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test failed !"  $NORMAL
    exit 1
fi

#HML

#ODS
rosrun arp_ods uTest_orders.ard
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test failed !"  $NORMAL
    exit 1
fi

#RLU 
rosrun arp_rlu uTest_CornerDetector.ard 
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test failed !"  $NORMAL
    exit 1
fi
rosrun arp_rlu uTest_ObjectFinder.ard
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test failed !"  $NORMAL
    exit 1
fi
rosrun arp_rlu uTest_LaserToolbox.ard
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test failed !"  $NORMAL
    exit 1
fi
rosrun arp_rlu uTest_ReLocalizator.ard
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test failed !"  $NORMAL
    exit 1
fi

#Master

