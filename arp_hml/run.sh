#!/bin/bash

. /opt/color.sh

if [[ $EUID -ne 0 ]]; then
	cecho red-e $ROUGE "This script must be run as root"
	exit 1
fi
cd `rospack find arp_hml`

ROOT_DEPLOYMENT_FILE="`rospack find arp_hml`/script/orocos/deployment/deploy_ubiquity.ops"
#ROOT_DEPLOYMENT_FILE="`rospack find arp_hml`/script/orocos/deployment/deploy_ubiquity_simul.ops"


if [ $# == 1 ]
then
	cecho yellow "You probably need to copy paste this into gdb :"
	cecho yellow "run -s $ROOT_DEPLOYMENT_FILE"
	gdb `rospack find ocl`/bin/deployer-$OROCOS_TARGET
else
	cecho yellow "running : rosrun ocl deployer-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE"
	rosrun ocl deployer-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE
fi


