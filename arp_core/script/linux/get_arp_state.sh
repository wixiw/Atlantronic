#!/bin/bash
if [ -f /tmp/ARP_LOADING ]
then
	echo "Loading..."
	exit
fi

if [ -f /tmp/ARP_FAILED ]
then
	echo "Deployment failed !!"
	exit
fi

if [ -f /tmp/ARP_DEPLOYED ]
then
	echo "Deployed"
	exit
fi

if [ -f /tmp/ARP_READY ]
then
	echo "Ready to fight !"
	exit
fi

if [ -f /tmp/ARP_RUNNING ]
then
	echo "Running"
	exit
fi

if [ -f /tmp/ARP_FINISHED ]
then
	echo "Finished"
	exit
fi

echo "Idle"