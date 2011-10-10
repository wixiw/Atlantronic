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

if [[ $EUID -ne 0 ]]; then
   echo -e $ROUGE "This script must be run as root" $NORMAL 1>&2
   exit 1
fi
cd `rospack find arp_core`
rosrun ocl deployer-$OROCOS_TARGET -s `rospack find arp_core`/script/orocos/deployment/deploy_arp_core.ops


