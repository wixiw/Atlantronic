#!/bin/bash
#18/04/2012
#version 10.0.0
#WLA
#
# this script check if a patch exist for the vm and update it automatically if needed

. /opt/color.sh 

rm /opt/kernel/update.sh -f

cd /opt/kernel
if [ $? != 0 ]; then
    cecho red "can't cd to /opt/kernel"
    exit 1
fi

wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Publication/livraison_v10.0/patch_10.1/update.sh
if [ $? != 0 ]; then
    cecho red "failed to get 10.1 patch, check network and ftp path !"
    exit 1
fi
dos2unix update.sh

#check root user
if [[ $UID -ne 0 ]]; then
	sudo bash update.sh
else
	bash update.sh
fi

