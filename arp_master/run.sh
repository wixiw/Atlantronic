#!/bin/bash

#
#choose your deployment file :
#
ROOT_DEPLOYMENT_FILE="`rospack find arp_master`/script/orocos/deployment/deploy_arp_master.ops"
ROOT_DEPLOYMENT_FILE_SIMUL="`rospack find arp_master`/script/orocos/deployment/deploy_arp_master_simul.ops"

#
# write "-corba" to use corba, else leave empty
#
CORBA="-corba"

roscore&
sleep 2
. /opt/color.sh

cd `rospack find arp_master`

if [[ $1 == "debug" ]]
then
	cecho yellow "You probably need to copy paste this into gdb :"
	cecho yellow "run -s $ROOT_DEPLOYMENT_FILE"
	gdb  `rosstack find orocos_toolchain`/install/bin/deployer$CORBA-$OROCOS_TARGET
else if [[ $1 == "simu" ]] 
	then
		cecho yellow "SIMU"
		cecho yellow "running : rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE_SIMUL"
		rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE_SIMUL
	else
		cecho yellow "running : rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE"
		rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE	
	fi
fi


