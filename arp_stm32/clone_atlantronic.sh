#!/bin/bash

cd /opt/ard/arp_stm32/src

if [ -e Atlantronic ]
then
	echo "Atlantronic is already checkouted."
	cd Atlantronic
	git pull
	exit 0
else
	echo "Atlantronic is not present yet."
	git clone https://github.com/wixiw/Atlantronic.git
	exit 0
fi
