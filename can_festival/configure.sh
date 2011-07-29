#!/bin/bash

if [ $# == 1 ]
then
	if [ $1 == "debug" ]
	then
	cd src
	./configure --timers=xeno --can=socket --enable-lss --enable-lss-fs --prefix=`rospack find can_festival` --debug=WAR
	fi
else
	cd src
	./configure --timers=xeno --can=socket --enable-lss --enable-lss-fs --prefix=`rospack find can_festival` --debug="ERR"
fi
