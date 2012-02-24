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
rosrun arp_core uTest.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test arp_core failed !"  $NORMAL
    exit 1
fi

#HML
rosrun arp_hml uTest.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test arp_hml failed !"  $NORMAL
    exit 1
fi

#IHM
rosrun arp_ihm uTest.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test arp_ihm failed !"  $NORMAL
    exit 1
fi

#ODS
rosrun arp_ods uTest.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test arp_ods failed !"  $NORMAL
    exit 1
fi

#RLU 
rosrun arp_rlu uTest.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test arp_rlu failed !"  $NORMAL
    exit 1
fi

#Master
rosrun arp_master uTest.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Test arp_master failed !"  $NORMAL
    exit 1
fi
