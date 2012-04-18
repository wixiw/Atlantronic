#!/bin/bash
#15/01/2011
#version 9.1.0
#WLA
#
# this script check if a patch exist for the vm and update it automatically if needed

VERT="\\033[1;32m" 
NORMAL="\\033[0;39m" 
ROUGE="\\033[1;31m" 
ROSE="\\033[1;35m" 
BLEU="\\033[1;34m" 
BLANC="\\033[0;02m" 
BLANCLAIR="\\033[1;08m" 
JAUNE="\\033[1;33m" 
CYAN="\\033[1;36m" 

rm /opt/kernel/update.sh -f

cd /opt/kernel
if [ $? != 0 ]; then
    echo -e $ROUGE "can't cd to /opt/kernel"  $NORMAL
    exit 1
fi

wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Publication/livraison_v9.0/patch_9.1/update.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "failed to get 9.1 patch, check network and ftp path !"  $NORMAL
    exit 1
fi
dos2unix update.sh

#check root user
if [[ $UID -ne 0 ]]; then
	sudo bash update.sh
else
	bash update.sh
fi


