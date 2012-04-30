#!/bin/bash -e
# auteur : WLA
# data : 30/04/2012
# version 10.0

# This script allows you to publish $ROS_PATH and $ROS_ADDONS_PATH files and the wixibox AFTER they have been compiled
# You have to be on an existing working system to have this to work

###############################################################################################################

WIXIBOX=88.191.124.77
ROS_PATH=/opt/ros-electric
ROS_ADDONS_PATH=/opt/ros_addons-electric
FTP_ROS_DEPENDENCY_DIR="34 - Info/Dependance/ROS"

###############################################################################################################
#source env datas
. /opt/color.sh
. /opt/env.sh

#if DEBUG env variable is set to something, use x option whihc prints each executed line
[ -n "$DEBUG" ] && set -x
set -e

#check root user
if [[ $UID -ne 0 ]]; then
	cecho red "$0 must be run as root"
	exit 1
fi 

#check tarballs name config
if [ ! -f $ROS_PATH/ard-version ] ; then
	cecho red "Please create a $ROS_PATH/ard-version file containing the name of the tgz to produce"
else
	ROS_TARBALL_NAME=`cat $ROS_PATH/ard-version`	
fi
if [ ! -f $ROS_ADDONS_PATH/ard-version ] ; then
	cecho red "Please create a $ROS_PATH/ard-version file containing the name of the tgz to produce"
else
	ROS_ADDONS_TARBALL_NAME=`cat $ROS_ADDONS_PATH/ard-version`
fi

#check wixibox alivness
cecho yellow "Checking wixibox aliveness..." 
cecho blue "Ping wixibox"
ping -c 1 -q $WIXIBOX > /dev/null

###############################################################################################################

cecho yellow "Removing temporary compilation files"
find $ROS_PATH -name build | xargs rm -rf
find $ROS_ADDONS_PATH  -name build | xargs rm -rf

cecho yellow "Strip binaries and libs"
set +e
strip $ROS_PATH/* 2>/dev/null
strip $ROS_ADDONS_PATH/* 2>/dev/null
set -e

cecho yellow "Clean execution outputs (log, graphs, telemetry)"
rosclean purge
find /opt/ard -name "*.log" | xargs rm -f
find /opt/ard -name "*.dat" | xargs rm -f
find /opt/ard -name "*.dot" | xargs rm -f
find /opt/ard -name "*.pid" | xargs rm -f
rm $ROS_PATH/*.log $ROS_PATH/*.pid $ROS_PATH/*.dat $ROS_PATH/*.dot -f

cecho yellow "Restore correct permissions"
chown ard:ard $ROS_PATH -R
chown ard:ard $ROS_ADDONS_PATH -R

cecho yellow "Creation of tarballs $ROS_TARBALL_NAME  and $ROS_ADDONS_TARBALL_NAME"

cd /opt
rm -Rf $ROS_TARBALL_NAME $ROS_ADDONS_TARBALL_NAME
tar -czf $ROS_TARBALL_NAME  ros-electric
tar -czf $ROS_ADDONS_TARBALL_NAME  ros_addons-electric

cecho yellow "Publication on Wixibox"
ftp -n -v $WIXIBOXHOST << EOT
user ard_user robotik
cd $FTP_ROS_DEPENDENCY_DIR
put $ROS_TARBALL_NAME
put $ROS_ADDONS_TARBALL_NAME
bye
EOT
