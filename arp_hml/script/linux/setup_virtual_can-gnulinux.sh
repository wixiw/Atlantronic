#!/bin/bash

#this script allows to load or unload can modules
#author : WLA

MODPATH=/lib/modules/`uname -r`/kernel
echo "MODPATH=$MODPATH   arg=$1"

if [[ $1 ]] 
then 
	echo ""	
else
	echo "Please add 'insert' or 'remove' argument to command line"
fi


if [ $1 = "stop" ] 
then
	echo "Removing can drivers..."
	echo "remove vcan"
	rmmod vcan
	echo "remove can_bcm"
	rmmod can_bcm
	echo "remove can_raw"
	rmmod can_raw
	echo "remove can"
	rmmod can
else
	if [ $1 = "start" ]  
	then
		echo "Inserting can drivers..."
		echo "insert can"
		modprobe can
		echo "insert can_raw"
		modprobe can_raw
		echo "insert can_bcm"
		modprobe can_bcm 
		echo "insert vcan"
		modprobe vcan

		echo "Configuring can ..."
		ip link add dev vcan0 type vcan
		ip link add dev vcan1 type vcan
		ip link set up vcan0
		ip link set up vcan1
		echo "done check ifconfig to see if vcan0 and vcan1 are up"
	else
		echo "Incorrect arguments, try 'start' or 'stop'"
	fi
fi


