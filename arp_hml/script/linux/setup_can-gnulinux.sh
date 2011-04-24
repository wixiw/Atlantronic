#!/bin/bash

#this script allows to load or unload can modules
#author : WLA

IRQ_NUMBER=5
MEM_HEX_ADDR=0xD0000
CAN_DEV_PATH=/dev
CAN_MAJOR=250
MODPATH=/lib/modules/`uname -r`/kernel

echo "MODPATH=$MODPATH   arg=$1"

if [[ $1 ]] 
then 
	echo ""	
else
	echo "Please add 'start' or 'stop' argument to command line"
	exit 0
fi

if [[ $2 ]] 
then 
	echo ""	
else
	echo "Please add 'scan' (socketCan drivers) or 'bci' (Ixxat drivers) argument to command line"
	exit 0
fi


if [ $1 = "stop" ] 
then
	echo "Removing can drivers ..."
	if [ $2 = "scan" ] 
	then
		echo "remove sja1000_isa"
		rmmod sja1000_isa
	else
		if [ $2 = "bci" ] 
		then
			echo "remove can_pas"
			rmmod can_pas
			echo "removing can dev nodes"
			sudo rm -f $CAN_DEV_PATH/can0
			sudo rm -f $CAN_DEV_PATH/can1
		else
			echo "Error"
			exit 0
		fi
	fi
	echo "remove sja1000"
	rmmod sja1000
	echo "remove can_raw"
	rmmod can_raw
	echo "remove can_bcm"
	rmmod can_bcm
	echo "remove can"
	rmmod can
else
	if [ $1 = "start" ]  
	then
		

		if [ $2 = "scan" ] 
		then
			echo "Inserting SocketCan drivers ..."
			modprobe can
			echo "insert can_raw"
			modprobe can_raw
			echo "insert can_bcm"
			modprobe can_bcm
			echo "insert sja1000"
			modprobe sja1000 
			echo "insert sja1000_isa (SocketCan driver)"
			insmod $MODPATH/drivers/net/can/sja1000/sja1000_isa.ko irq=$IRQ_NUMBER mem=$MEM_HEX_ADDR

			echo "Configuring SocketCan interfaces ..."
			ip link set can0 type can bitrate 250000 restart-ms 1000
			#ip link set can1 type can bitrate 250000 restart-ms 1000
			ifconfig can0 up
			#ifconfig can1 up
			echo "done, check ifconfig to see if can0 is present"
		else
			if [ $2 = "bci" ] 
			then
				echo "Inserting Ixxat BCI drivers ..."
				modprobe can
				echo "insert can_raw"
				modprobe can_raw
				echo "insert can_bcm"
				modprobe can_bcm
				echo "insert sja1000"
				modprobe sja1000 
				echo "insert can_pas (BCI Ixxat driver)"
				modprobe can_pas addr=$MEM_HEX_ADDR irq=$IRQ_NUMBER type=03ISA major=$CAN_MAJOR

				echo "Configuring Ixxat BCI device nodes ..."
				if [ ! -c $CAN_DEV_PATH/can0 ]; then sudo mknod $CAN_DEV_PATH/can0 c $CAN_MAJOR 0; fi
				#if [ ! -c $CAN_DEV_PATH/can1 ]; then sudo mknod $CAN_DEV_PATH/can1 c $CAN_MAJOR 1; fi
				sudo chmod a+rw $CAN_DEV_PATH/can*
				echo "done, check /dev/can0"
			else
				echo "Error"
				exit 0
			fi
		fi
	else
		echo "Error"
		exit 0
	fi
fi


