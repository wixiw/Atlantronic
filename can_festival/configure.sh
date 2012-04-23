#!/bin/bash

#if [ $OROCOS_TARGET == "xenomai" ]
"then
#	TIMERS="xeno"
#fi
#if [ $OROCOS_TARGET == "gnulinux" ]
#then
#	TIMERS="unix"
#fi

TIMERS="xeno"

if [ $# == 1 ]
then
	if [ $1 == "debug" ]
	then
	cd src
	./configure --timers=$TIMERS --can=socket --enable-lss --enable-lss-fs --prefix=`rospack find can_festival` --debug=WAR
	fi
else
	cd src
	./configure --timers=$TIMERS --can=socket --enable-lss --enable-lss-fs --prefix=`rospack find can_festival` --debug="ERR"
fi
