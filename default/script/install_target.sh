#!/bin/bash

# author : WLA
# date : 04/03/2011
#
# this script allows to synchronize the robot with the dev folders. 
# The aim is to have the robot folder equal to the dev folder.
# "CAUTION" : this script will erase everything that is not on the dev side,
# so If you have  done "bricolages" on the target they'll be erased !
#
# USER : 
# this script has 2 arguments :
# - source_project : the name of the ros stack or package to synchronize
# - target_ip : the ID adress of the robot
#
# examples : 
# sh install.sh ard 192.168.1.29
# sh install.sh arp_core 192.168.1.29
#

HOSTNAME=`cat /etc/hostname`

VERT="\\033[1;32m" 
NORMAL="\\033[0;39m" 
ROUGE="\\033[1;31m" 
ROSE="\\033[1;35m" 
BLEU="\\033[1;34m" 
BLANC="\\033[0;02m" 
BLANCLAIR="\\033[1;08m" 
JAUNE="\\033[1;33m" 
CYAN="\\033[1;36m" 

package_name=$1
ip_adress=$2

#check inputs
if [ $# != 2 ]
then
	echo -e $ROUGE "[!] you did not provide correct arguments. Please enter a package_name and an ip adress" $NORMAL
echo -e $ROUGE "example : sh install.sh ard 192.168.1.29" $NORMAL
	exit 0
fi


#check that you are not already on the target
if [ $HOSTNAME == "alpha" ]
then
	echo -e $BLEU "[!] you are on the target !" $NORMAL
	exit 0
fi
if [ $HOSTNAME == "beta" ]
then
	echo -e $BLEU "[!] you are on the target !" $NORMAL
	exit 0
fi

#check IP connexion
echo -e $BLEU  "Ping $ip_adress" $NORMAL
ping -c 1 -q $ip_adress > /dev/null
if [ $? != 0 ]; then
    echo -e $ROUGE "$ip_adress not reachable, check network !"  $NORMAL
    exit 1
fi

#synchronize datas
echo -e $BLEU
if [ $package_name == "ard" ]
then
	rsync  -avzh `rosstack find ard` root@$2:/opt/ros \
	--delete \
	--exclude "build" \
	--exclude ".svn" \
	--exclude ".git" \
	--exclude ".hg" \
	--exclude ".tb_history"\
	--exclude "*.log"\
	--exclude "doc"
else
	rsync  -avzh `rospack find $package_name` root@$2:`rosstack find ard` \
	--delete \
	--exclude "build" \
	--exclude ".svn"  \
	--exclude ".git" \
	--exclude ".hg"\
	--exclude ".tb_history"\
	--exclude "*.log" \
	--exclude "doc"
fi
echo -e $NORMAL 

#check success of rsync
if [ $? != 0 ]; then
    echo -e $ROUGE "[!] Syncronization failed !"  $NORMAL
    exit 1
else
	echo -e $VERT "[.] rsync succeed" $NORMAL
fi

date
