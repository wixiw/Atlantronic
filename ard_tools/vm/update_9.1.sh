#!/bin/bash
IS_HOST="true"
. /opt/color.sh

#define target dependant variables
if [ $HOSTNAME == "alpha" ]; then
        cecho blue "[!] you are on the alpha target !"
	IS_HOST="false"
else 
	if [ $HOSTNAME == "beta" ]; then
		cecho blue "[!] you are on the beta target !"
		IS_HOST="false"
	else
		cecho blue "[!] you are on the VM target !"
	fi
fi

#verification des versions
VM_CURRENT_VERSION=`cat /opt/kernel/ard-vm-version`
if [ $VM_CURRENT_VERSION == "9.1" ]; then
	cecho green "VM already in version 9.1, which is the last version"
	exit 0
fi
if [ $VM_CURRENT_VERSION != "9.0" ]; then
	cecho red "VM is in an unexpected version $VM_CURRENT_VERSION"
	exit 1
fi

#suppression de lua5.0 qui empeche rFSM de fonctionner
apt-get -y remove liblua50
if [ $? != 0 ]; then
    cecho red "Failed to remove lua5.0"
    exit 1
fi

#installation des lib oubliees
apt-get -y install libomniorb4-1 lua5.1 setserial python-serial stress
apt-get -y autoremove 
if [ $? != 0 ]; then
    cecho red "Failed to install dependencies"
    exit 1
fi
echo "
alias ard-stress='stress --cpu 2 --io 1 --vm 1 --vm-bytes 128M'
" >> /opt/env.sh

#recuperation du logo de boot et de l'ecran de fond
if [ $IS_HOST == "true" ]; then
	IMG_PATH="/home/ard/Pictures"
else
	IMG_PATH="/root"
fi
cd $IMG_PATH
wget ftp://ard_user:robotik@88.191.124.77/0%20-%20Docs%20de%20ref/0%20-%20Medias/Logo/Logo-ARD_800_600_fonce.jpg
if [ $? != 0 ]; then
    cecho red "Failed to download boot screen"
    exit 1
fi
echo "
#wallpaper pour ARD
GRUB_BACKGROUND=\"$IMG_PATH/Logo-ARD_800_600_fonce.jpg\"
" >> /etc/default/grub
if [ $IS_HOST == "true" ]; then
	wget ftp://ard_user:robotik@88.191.124.77/0%20-%20Docs%20de%20ref/0%20-%20Medias/Wallpaper/wallpaper_vm9.jpg
	if [ $? != 0 ]; then
	    cecho red "Failed to download wallpaper"
	    exit 1
	fi
fi


#ajout des bips de démarrage
sed -i "s|log_action_msg \"Will now halt\"|#beeps;\r\n\t/usr/bin/beep -f 300 -l 300\r\n\t/usr/bin/beep -f 285 -l 300\r\n\t/usr/bin/beep -f 270 -l 500\r\n\r\n\tlog_action_msg \"Will now halt\"|" /etc/init.d/halt 
if [ $? != 0 ]; then
    cecho red "Failed to add halt bips"
    exit 1
fi
sed -i "s|log_action_msg \"Will now restart\"|\r\n\t#beeps;\r\n\t/usr/bin/beep -f 300 -l 300\r\n\t/usr/bin/beep -f 285 -l 300\r\n\t/usr/bin/beep -f 270 -l 300\r\n\t/usr/bin/beep -f 300 -l 300\r\n\r\n\tlog_action_msg \"Will now restart\"|" /etc/init.d/reboot 
if [ $? != 0 ]; then
    cecho red "Failed to add reboot bips"
    exit 1
fi

echo "
#son pour ARD
GRUB_INIT_TUNE=\"360 270 1 285 1 300 1\"
" >> /etc/default/grub



#recuperation des nouveaux noyaux 
cd /opt/kernel
if [ $IS_HOST == "true" ]; then
	wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Dependance/Linux/OS_VM/v9/linux-image-2.6.38.8-vm-9.04_0_i386.deb
	if [ $? != 0 ]; then
	    cecho red "Failed to download linux image"
	    exit 1
	fi

	wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Dependance/Linux/OS_VM/v9/linux-headers-2.6.38.8-vm-9.04_0_i386.deb
	if [ $? != 0 ]; then
	    cecho red "Failed to download linux headers"
	    exit 1
	fi
else
	wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Dependance/Linux/OS_Target/v9/linux-image-2.6.38.8-target-9.04_0_i386.deb
	if [ $? != 0 ]; then
	    cecho red "Failed to download target linux image"
	    exit 1
	fi
fi
dpkg -i linux-*-2.6.38.8-*-9.04_0_i386.deb
if [ $? != 0 ]; then
    cecho red "Failed to install new kernel packages"
    exit 1
fi
cd /lib/modules/2.6.38.8-vm-9.04
ln -sf /usr/src/linux-headers-2.6.38.8-vm-9.04 build
if [ $? != 0 ]; then
    cecho red "Failed to create /lib/module symbolic build link"
    exit 1
fi
ln -sf /usr/src/linux-headers-2.6.38.8-vm-9.04 sources
if [ $? != 0 ]; then
    cecho red "Failed to create /lib/module symbolic sources link"
    exit 1
fi


#installation des stack manquantes pour les dynamixel : nouvelle version des trucs ros
cd /opt
if [ -d /opt/ros-electric ]; then 
	tar -czf ros-electric_pre-patch9.1_bak.tgz ros-electric
	if [ $? != 0 ]; then
	    cecho red "Failed to save old ros-electric install"
	    exit 1
	fi
fi
if [ -d /opt/ros_addons-electric ]; then
	tar -czf ros_addons-electric_pre-patch9.1_bak.tgz ros_addons-electric
	if [ $? != 0 ]; then
	    cecho red "Failed to save old ros_addons-electric install"
	    exit 1
	fi
fi
rm -Rf ros-electric ros_addons-electric
if [ $? != 0 ]; then
    cecho red "Failed to delete old ros folders"
    exit 1
fi
if [ $IS_HOST == "true" ]; then
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/ROS/ros-electric_ard-9.1_i386.tgz
	if [ $? != 0 ]; then
	    cecho red "Failed to download ros-electric_9.1"
	    exit 1
	fi
	tar -xf ros-electric_ard-9.1_i386.tgz
	if [ $? != 0 ]; then
	    cecho red "Failed to extract ros-electric_9.1"
	    exit 1
	fi
	echo "ros-electric_ard-9.1_i386.tgz" > ros-electric/ard-version
	chown ard:ard /opt/ros-electric -R
	if [ $? != 0 ]; then
	    cecho red "Failed to give ownership back to ard"
	    exit 1
	fi
fi
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/ROS/ros_addons-electric_ard-9.1_i386.tgz
if [ $? != 0 ]; then
    cecho red "Failed to download ros_addons-electric_9.1"
    exit 1
fi
tar -xf ros_addons-electric_ard-9.1_i386.tgz
if [ $? != 0 ]; then
    cecho red "Failed to extract ros_addons-electric_9.1"
    exit 1
fi
echo "ros_addons-electric_ard-9.1_i386.tgz" > ros_addons-electric/ard-version
chown ard:ard /opt/ros_addons-electric -R
if [ $? != 0 ]; then
    cecho red "Failed to give ownership back to ard"
    exit 1
fi

#suppression du probleme avec exim4
sed -i "s|dc_local_interfaces='127.0.0.1 ; ::1'|dc_local_interfaces='127.0.0.1'|" /etc/exim4/update-exim4.conf.conf
rm /var/log/exim4/paniclog

#preparation du prochain patch
cd /opt/kernel
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Publication/livraison_v9.0/patch_9.1/check_update-9.1.0.sh
if [ $? != 0 ]; then
    cecho red "Failed to get next patch check"
    exit 1
fi
ln -sf check_update-9.1.0.sh check_update.sh
dos2unix check_update-9.1.0.sh
if [ $? != 0 ]; then
    cecho red "Failed to convert dos2unix"
    exit 1
fi

#recuperation des notes de version
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Publication/livraison_v9.0/patch_9.1/release-note-9.1.txt
if [ $? != 0 ]; then
    cecho red "Failed to get notes"
    exit 1
fi

#succes
cecho green "SUCCEED !!!"
cecho yellow `cat release-note-9.1.txt`
cecho green "please reboot and reinstall VM additions and then retry ard-update to check next update"
echo "9.1" > /opt/kernel/ard-vm-version 
