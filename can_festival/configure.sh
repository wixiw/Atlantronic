#!/bin/bash

if [[ $CAN_FLAVOR == "xenomai" ]]
then
	echo "ARD : CanFestival will be compiled against xenomai timers"
	TIMERS="xeno"
fi
if [[ $CAN_FLAVOR == "gnulinux" ]]
then
	echo "ARD : CanFestival will be compiled against gnulinux timers"
	TIMERS="unix"
fi


if [ $# == 1 ]
then
	if [ $1 == "debug" ]
	then
	cd src
	./configure --timers=$TIMERS --can=socket --enable-lss --enable-lss-fs --prefix=`rospack find can_festival` --debug=ERR
	fi
else
	cd src
	./configure --timers=$TIMERS --can=socket --enable-lss --enable-lss-fs --prefix=`rospack find can_festival` --debug=ERR
fi
