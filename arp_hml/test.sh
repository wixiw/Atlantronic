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

if [ $# == 1 ]
then
	echo -e $JAUNE "You probably need to copy paste this into gdb :" $NORMAL
	echo -e $JAUNE "run -s script/orocos/deployment/deploy_protokrot.ops" $NORMAL
	gdb `rospack find ocl`/install/bin/deployer-gnulinux
else
	rosrun ocl deployer-gnulinux -s script/orocos/deployment/deploy_tests.ops	
	#rosrun ocl deployer-gnulinux -s script/orocos/deployment/deploy_hml_simul.ops	
fi


