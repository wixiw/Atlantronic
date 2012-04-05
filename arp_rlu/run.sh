#!/bin/bash

#
#choose your deployment file :
#
ROOT_DEPLOYMENT_FILE="`rospack find arp_rlu`/script/orocos/deployment/deploy_arp_rlu.ops"
ROOT_DEPLOYMENT_FILE_SIMU="`rospack find arp_rlu`/script/orocos/deployment/deploy_arp_rlu_simul.ops"

#
# write "-corba" to use corba, else leave empty
#
CORBA="-corba"


. /opt/color.sh

cd `rospack find arp_rlu`

if [ $1 == "simu" ]
then
	cecho yellow "SIMU"
	cecho yellow "running : rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE_SIMU"
	rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE_SIMU
else
	cecho yellow "running : rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE"
	rosrun ocl deployer$CORBA-$OROCOS_TARGET -s $ROOT_DEPLOYMENT_FILE
fi
