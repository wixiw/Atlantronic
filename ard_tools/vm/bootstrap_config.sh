#!/bin/bash -ex
# auteur : WLA
# data : 18/04/2012
# version 9.3

# This script contains configuration variablesfor the bootstrap_ard.sh script.

HOSTNAME=`cat /etc/hostname`

ARCH="i386"
ARD_VERSION="9.0"
ALPHA_IP=192.168.1.29
BETA_IP=192.168.1.41
VM_IP=192.168.1.20
IS_HOST="true"
JOBS="-j3"
ROS_DISTRIBUTION="electric"
	#ROS_VERSION="ros_${ROS_DISTRIBUTION}_ard-"$ARD_VERSION"_"$ARCH".tgz"
	#ROS_ADDONS_VERSION="ros_addons_${ROS_DISTRIBUTION}_ard-"$ARD_VERSION"_"$ARCH".tgz"
ROS_VERSION="ros_${ROS_DISTRIBUTION}_ard-8.2_"$ARCH".tgz"
ROS_ADDONS_VERSION="ros_addons_${ROS_DISTRIBUTION}_ard-8.2_"$ARCH".tgz"
ECLIPSE_VERSION="eclipse_indigo-ard_"$ARD_VERSION"_"$ARCH".tgz"
WIXIBOX=88.191.124.77
KERNEL_SUBDIR="v8"
KERNEL_BASE_VERSION="2.6.38.8"
KERNEL_VM_VERSION="$KERNEL_BASE_VERSION-vm-8.57"
KERNEL_TARGET_VERSION="$KERNEL_BASE_VERSION-target-8.57"
WALLPAPER_PATH="34%20-%20Info/Publication/wallpaper_ARD_2.jpg"
XENOMAI_VERSION="2.6.0"
XENOMAI_ARD_VERSION=$XENOMAI_VERSION"-ard1"

TARGET_DEBIAN_PACKAGES="mc gdb rsync localepurge deborphan debfoster python-setuptools python-yaml subversion git mercurial lm-sensors python-paramiko python-numpy ccze beep dos2unix ruby ccze valgrind apache2 mysql-server phpmyadmin sudo libboost-program-options1.42.0  libboost-serialization1.42.0 libboost-filesystem1.42.0  libboost-thread1.42.0 libboost-signals1.42.0 libboost-regex1.42.0 libreadline6 liblua5.1-0 omniorb4 omniidl4 omniorb4-nameserver libomniorb4-1 liblog4cxx10"
VM_DEBIAN_PACKAGES=$TARGET_DEBIAN_PACKAGES" filezilla cmake cmake-curses-gui python python-wxgtk2.8 python-qt4 python-wxtools wx2.8-i18n libwxgtk2.8-dev libgtk2.0-dev doxygen graphviz kst meld kernel-package ketchup gitk openjdk-6-jre sloccount dkms libsvn-java python-scipy python-qt4 magicfilter ccache distcc distcc-pump distccmon-gnome xpdf"


#define target dependant variables
IP_ADDRESS=$VM_IP
if [ $HOSTNAME == "alpha" ]; then
        echo -e $BLEU "[!] you are on the alpha target !" $NORMAL
	IP_ADDRESS=$ALPHA_IP
	IS_HOST="false"
	JOBS="-j2"
else 
	if [ $HOSTNAME == "beta" ]; then
		echo -e $BLEU "[!] you are on the beta target !" $NORMAL
		IP_ADDRESS=$BETA_IP
		IS_HOST="false"
		JOBS="-j2"
	else
		echo -e $BLEU "[!] you are on the VM target !" $NORMAL
	fi
fi

#choix de l'editor de fichier
if [ $IS_HOST == "true" ]; then
	EDITOR="gedit"
else
	EDITOR="nano"
fi
