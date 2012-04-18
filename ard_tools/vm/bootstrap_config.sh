#!/bin/bash -ex
# auteur : WLA
# data : 18/04/2012
# version 10.0

# This script contains configuration variablesfor the bootstrap_ard.sh script.

HOSTNAME=`cat /etc/hostname`

NORMAL="\\033[0;39m"
BLEU="\\033[1;34m"

ARCH="i386"
ARD_VERSION="10.0"
ALPHA_IP=10.0.0.29
BETA_IP=10.0.0.41
VM_IP=10.0.0.20
ACKSYS_VM_IP=10.0.0.253
WIXIBOX=88.191.124.77

IS_HOST="true"
JOBS="-j3"
ROS_DISTRIBUTION="electric"
ROS_VERSION="ros_${ROS_DISTRIBUTION}_ard-"$ARD_VERSION"_"$ARCH".tgz"
ROS_ADDONS_VERSION="ros_addons_${ROS_DISTRIBUTION}_ard-"$ARD_VERSION"_"$ARCH".tgz"
ECLIPSE_VERSION="eclipse_indigo-ard_"$ARD_VERSION"_"$ARCH".tgz"
KERNEL_SUBDIR="v10"
KERNEL_BASE_VERSION="2.6.38.8"
KERNEL_VM_VERSION="$KERNEL_BASE_VERSION-vm-10.0"
KERNEL_TARGET_VERSION="$KERNEL_BASE_VERSION-target-10.0"
XENOMAI_VERSION="2.6.0"
XENOMAI_ARD_VERSION=$XENOMAI_VERSION"-ard1"

BOOST_PACKAGES="libboost-program-options1.42.0  libboost-serialization1.42.0 libboost-filesystem1.42.0  libboost-thread1.42.0 libboost-signals1.42.0 libboost-regex1.42.0"
OROCOS_PACKAGES="libreadline6 liblua5.1-0 lua5.1 omniorb4 omniidl4 omniorb4-nameserver libomniorb4-1 liblog4cxx10"
TOOLS_PACKAGES="mc rsync gdb localepurge deborphan debfoster subversion git mercurial ccze beep dos2unix valgrind sudo setserial stress"
WEB_PACKAGES="apache2 mysql-server phpmyadmin"
TARGET_DEBIAN_PACKAGES=$BOOST_PACKAGES" "$OROCOS_PACKAGES" "$TOOLS_PACKAGES" "$WEB_PACKAGES" lm-sensors ruby python-setuptools python-yaml python-paramiko python-numpy python-serial"

VM_TOOLS_PACKAGES="sshfs filezilla cmake cmake-curses-gui kst meld kernel-package ketchup gitk sloccount magicfilter ccache distcc distcc-pump distccmon-gnome xpdf git-core"
VM_PYTHON_PACKAGES="python python-wxgtk2.8 python-qt4 python-wxtools python-scipy python-qt4 python-pip"
VM_DEBIAN_PACKAGES=$TARGET_DEBIAN_PACKAGES" "$VM_TOOLS_PACKAGES" "$VM_PYTHON_PACKAGES" libzzip-0-13 libomnithread3-dev omniidl omniorb omniorb-nameserver libomniorb4-dev wx2.8-i18n libwxgtk2.8-dev libgtk2.0-dev doxygen graphviz  openjdk-6-jre dkms libsvn-java"
 
GRUB_INIT_SOUND="360 270 1 285 1 300 1"

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
