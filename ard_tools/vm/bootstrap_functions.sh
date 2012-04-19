#!/bin/bash
# auteur : WLA
# data : 18/04/2012
# version 10.0
#
# Ce script contient les fonctions qui seront appelées lors du bootstrap

set -e

###
# use this to check if the wixibox servcies are available
# at present time respondign to a ping is enougth
function check_wixibox
{
	cecho yellow "Checking wixibox aliveness..." 
	cecho blue "Ping wixibox"
	ping -c 1 -q $WIXIBOX > /dev/null
}

###
# create default empty folders needed later
function create_folders
{
	cecho yellow "Creating default dirs..." 
	mkdir /home/ard/src -p
	mkdir /opt/kernel -p
	mkdir /unionfs -p	 
	mkdir /unionfs/etc -p
	mkdir /unionfs/var -p
	mkdir /unionfs/opt -p
	mkdir /unionfs/tmp -p
}

###
# 
function populated_rc.local
{
	cecho yellow "Populates rc.local..." 
	cp /etc/rc.local /etc/rc.local.origin 
	echo "#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will exit 0 on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.
	
#update the Message Of The Day each boot
echo \"---------------------------------------------------------------------
Linux $HOSTNAME `uname -r` # `date` i686\" > /etc/motd

udevadm control --reload-rules
udevadm trigger

#beeping to say hello
beep -f 300 -l 500
exit 0" > /etc/rc.local

}

###
# 
function configure_apt
{
	cecho yellow "Configuring APT..."
	cp /etc/apt/sources.list /etc/apt/sources.list.origin 
	echo "deb http://debian.ens-cachan.fr/ftp/debian/ squeeze main contrib non-free
	deb-src http://debian.ens-cachan.fr/ftp/debian/ squeeze main contrib non-free

	# squeeze-updates, previously known as 'volatile'
	deb http://debian.ens-cachan.fr/ftp/debian/ squeeze-updates main contrib
	deb-src http://debian.ens-cachan.fr/ftp/debian/ squeeze-updates main contrib
	
	deb http://security.debian.org/ squeeze/updates main contrib
	deb-src http://security.debian.org/ squeeze/updates main contrib" > /etc/apt/sources.list
	
	apt-get update -f
}

###
#
function configure_boot
{
	cecho yellow  "Configuring grub.."
	
	#changing the timeout. 0 would deasactivate it but it's a bit dangerous
	sed -i "s/GRUB_TIMEOUT=5/GRUB_TIMEOUT=1/" /etc/default/grub

	#add serial console in target
	#add no apic to vm to let them boot properly, *DON'T* do that for target else caninterrupts won't come any more.
	#choose the ro init script on target.
	if [ $IS_HOST == "true" ]; then
		sed -i "s/GRUB_CMDLINE_LINUX_DEFAULT=\"quiet\"/GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash noapic\"/" /etc/default/grub
	else
		sed -i "s|GRUB_CMDLINE_LINUX_DEFAULT=\"quiet\"|GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash init=/sbin/init_ro console=tty1 console=ttyS0,115200\"|" /etc/default/grub
		
		echo "serial --unit=1 --speed=115200"  >> /etc/grub.d/00_header
		echo "terminal serial" >> /etc/grub.d/00_header
	fi
	
	#change boot screen size
	sed -i "s/#GRUB_GFXMODE=640x480/GRUB_GFXMODE=800x600/" /etc/default/grub

	#disable reference to hard disk uuid which mess up sometimes
	sed -i "s/#GRUB_DISABLE_LINUX_UUID='true'/GRUB_DISABLE_LINUX_UUID='true'/" /etc/default/grub
	
	#disable recovery mode (we don't wear hockey pads)
	sed -i "s/#GRUB_DISABLE_LINUX_RECOVERY=\"true\"/GRUB_DISABLE_LINUX_RECOVERY=\"true\"/" /etc/default/grub

	#little bip to say 'I'm booting'
	sed -i "s/#GRUB_INIT_TUNE=\"480 440 1\"/GRUB_INIT_TUNE=\"$GRUB_INIT_SOUND\"/" /etc/default/grub
	
	#suppression du checkdisk (seulement sur les targets)
	if [ $IS_HOST == "false" ]; then
		cecho yellow "Suppression du checkdisk au boot..."
		sed -i "s/\/               ext2    errors=remount-ro 0       1/\/               ext2    errors=remount-ro 0       0/" /etc/fstab
	fi
	
	#correction de la ligne cdrom
	echo "/dev/hdc	/media/cdrom0	udf,iso9660	user,noauto	0	0" >> /etc/fstab
	
	#creation du repertoire de conf pour ce qui est lie a l'OS
	cecho yellow "Creating OS conf folder"
	mkdir /opt/conf -p
	echo "You *may* create empty files here to configure the way ARD's OS load :
	_ /opt/conf/MATCH : autostart main binairies for matches. 
	_ /opt/conf/RO : load a read only filesystem to protect the flash
	
	You *must* have a /opt/conf/OS containing either 'gnulinux' or 'xenomai' to select Orocos flavor." > /opt/conf/README
	echo "gnulinux" > /opt/conf/OS
	
##
# TODO :  get sbin_init
#faire un chmod +x dessus
}

###
# SYA is a simple recovery mode that allow to do things even if the target is in bad mood
# TODO : revoir SYA, je pense qu'il marche plus. Revoir aussi les insmod ext2 peut être outdated
function create_SYA
{
	if [ $IS_HOST == "true" ]; then
	echo "#!/bin/sh -e
	echo 'ARD Save Your Ass'
	cat << EOF
		menuentry 'ARD rescue system : SYA (Save Your Ass)' --class debian --class gnu-linux --class gnu --class os {
			insmod part_msdos
	        	insmod ext2
	        	set root='(hd0,msdos1)'
	        	search --no-floppy --fs-uuid --set afd918ab-16a6-46e9-a3b3-7712969747d4
	        	echo    'Chargement de Linux $KERNEL_VM_VERSION ...'
	        	linux   /boot/vmlinuz-$KERNEL_VM_VERSION root=/dev/sda1 init=/bin/bash
		}" > /etc/grub.d/39_ARD_SYA
	else
		echo "#!/bin/sh -e
	
	echo 'ARD Save Your Ass'
	cat << EOF
	menuentry 'ARD rescue system : SYA (Save Your Ass)' --class debian --class gnu-linux --class gnu --class os {
			insmod part_msdos
	        	insmod ext2
	        	set root='(hd0,msdos1)'
	        	search --no-floppy --fs-uuid --set f3b795cd-c804-4e71-9e1d-8a233722c71f
	        	echo    'Chargement de Linux $KERNEL_TARGET_VERSION ...'
	        	linux   /boot/vmlinuz-$KERNEL_TARGET_VERSION root=/dev/sda1 init=/bin/bash
		}" > /etc/grub.d/39_ARD_SYA
	fi
	
	
	chmod +x /etc/grub.d/39_ARD_SYA
	
	echo "#run this script in SYA mode to remount L in read write mode. You can try the last commented line to change the keyboard language
	mount -w -o remount /
#dpkg-reconfigure console-data" > /opt/kernel/sya.sh
}

###
#
function install_kernel
{
	#récupération des sources du noyau
	cecho yellow "Récupération des sources du noyau $KERNEL_BASE_VERSION ..."
	if [ $IS_HOST == "true" ]; then
		cd /usr/src
		mkdir linux-$KERNEL_BASE_VERSION -p
		ln -sf /usr/src/linux-$KERNEL_BASE_VERSION linux
		cd linux
		ketchup -G $KERNEL_BASE_VERSION
	fi
	
	cd /tmp
	CURRENT_KERNEL_VERSION=`uname -r`
	if [ $IS_HOST == "true" ]; then
		WANTED_KERNEL_VERSION=$KERNEL_VM_VERSION
	else
		WANTED_KERNEL_VERSION=$KERNEL_TARGET_VERSION
	fi
	
	if [ $CURRENT_KERNEL_VERSION == $WANTED_KERNEL_VERSION ]; then
		cecho green $VERT "Noyau ARD $KERNEL_BASE_VERSION already installed, skipping" 
	else
		cecho yellow "Installation du noyau ARD $KERNEL_BASE_VERSION ..."
		KERNEL_DL_PATH="ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Linux/OS_VM/$KERNEL_SUBDIR/linux-image-"${KERNEL_VM_VERSION}"_0_$ARCH.deb"
		KERNEL_HEADERS_DL_PATH="ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Linux/OS_VM/$KERNEL_SUBDIR/linux-headers-"${KERNEL_VM_VERSION}"_0_$ARCH.deb"
		if [ $IS_HOST == "false" ]; then
			KERNEL_DL_PATH="ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Linux/OS_Target/$KERNEL_SUBDIR/linux-image-"${KERNEL_TARGET_VERSION}"_0_$ARCH.deb"
			KERNEL_HEADERS_DL_PATH="ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Linux/OS_Target/$KERNEL_SUBDIR/linux-headers-"${KERNEL_TARGET_VERSION}"_0_$ARCH.deb"
		fi
		cecho yellow "Kernel from : $KERNEL_DL_PATH"
		cecho yellow "Kernel from : $KERNEL_HEADERS_DL_PATH"
	
		wget -c $KERNEL_DL_PATH
	
		#on ne prend les headers que sur le pc de dev
		if [ $IS_HOST == "true" ]; then
			wget -c $KERNEL_HEADERS_DL_PATH
		fi
	
		dpkg -i linux-*.deb
		mv *.deb /opt/kernel
	fi
	
	#correction des liens symboliques pour les sources noyau
	if [ $IS_HOST == "true" ]; then
		cecho yellow "Correcting VM kernel modules links"
		cd /lib/modules/$KERNEL_VM_VERSION
		rm build
		rm sources
		ln -sf /usr/src/linux-headers-$KERNEL_VM_VERSION build
		ln -sf /usr/src/linux-headers-$KERNEL_VM_VERSION sources
	fi
}


###
# This function configures 2 network interfaces :
# _ a LAN dhcp client for dev network
# _ a WLAN static IP on 10.0.0.X to manipulate the robot
function configure_network
{
	cecho yellow "Configuration du réseau ..."
	
	if [ $IS_HOST == "true" ]; then
		#ajout de l'IP fixe
		cecho yellow "Définition du double réseau vm..."
		echo "# This file describes the network interfaces available on your system
		# and how to activate them. For more information, see interfaces(5).
		# The loopback network interface
		auto lo
		iface lo inet loopback
		
		# The primary LAN DHCP network interface
		allow-hotplug eth0
		iface eth0 inet dhcp
		auto eth0
	
		# The secondary robot WLAN interface
		allow-hotplug wlan_robot
		iface wlan_robot inet static
		address $IP_ADDRESS
		netmask 255.255.255.0
		auto wlan_robot" > /etc/network/interfaces
	else
		#ajout de l'IP fixe
		cecho yellow "Définition du réseau IP fixe target..."
		echo "# This file describes the network interfaces available on your system
		# and how to activate them. For more information, see interfaces(5).
		# The loopback network interface
		auto lo
		iface lo inet loopback
		
		# The primary LAN DHCP network interface
		allow-hotplug eth0
		iface eth0 inet static
		address $IP_ADDRESS
		netmask 255.255.255.0
		auto eth0" > /etc/network/interfaces
	fi
	

	#configuration des addresses réseau
	echo "
	127.0.0.1       vm      arp_ihm
	$ALPHA_IP       alpha.team-ard.com      alpha
	$BETA_IP        beta.team-ard.com       beta
	$ACKSYS_VM_IP   acksys   acksys
	$WIXIBOX	    wixibox

	" >> /etc/hosts

	#modifying DNS timeout
	echo "timeout:1" >> /etc/resolvconf/resolv.conf.d/tail
	if [ $IS_HOST == "false" ]; then
		echo "" > /etc/resolvconf/resolv.conf.d/original
	fi
}

###
#
function install_osdeps
{
	#choix des paquets debians a installer
	if [ $IS_HOST == "true" ]; then
		PAQUET_LIST=$VM_DEBIAN_PACKAGES
	else
		PAQUET_LIST=$TARGET_DEBIAN_PACKAGES
	fi
	
	#installation des paquets manquants
	cecho -yellow "Installation des paquets manquants ..."
	cecho blue "liste des paquets a installer : $PAQUET_LIST"
	apt-get update --fix-missing
	apt-get install $PAQUET_LIST -y
	
	#mise a jour de pip
	sudo pip install -U pip
}


###
#
function install_xenomai
{
	cd /tmp
	cecho yellow "Installation de Xenomai $XENOMAI_ARD_VERSION ..."
	if [ $IS_HOST == "true" ]; then
		wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Dependance/Xenomai/xenomai-$XENOMAI_ARD_VERSION/libxenomai-dev_${XENOMAI_ARD_VERSION}_$ARCH.deb
	fi
	wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Dependance/Xenomai/xenomai-$XENOMAI_ARD_VERSION/libxenomai1_${XENOMAI_ARD_VERSION}_$ARCH.deb
	wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Dependance/Xenomai/xenomai-$XENOMAI_ARD_VERSION/xenomai-runtime_${XENOMAI_ARD_VERSION}_$ARCH.deb
	dpkg -i *xenomai*.deb
	mv *.deb /opt/kernel
}

###
#
function configure_version_systems
{
	#configuration de git
	git config --global color.diff auto
	git config --global color.status auto
	git config --global color.branch auto
	git config --global user.name "A.R.D."
	git config --global user.email contact@team-ard.com
}

###
#
function configure_user_profiles
{
	#configuration des consoles pour tous les utilisateurs
	cecho yellow "Configuration des profils utilisateurs ..."
	echo "#ARP env
	ROUGE='\\033[1;31m'
	NORMAL='\\033[0;39m'
	. /opt/env.sh
	if [ \$? != 0 ]; then
	    echo -e \$ROUGE 'Failed to load ARD env variables. Check if you /opt folder contains the rigth stuff'  \$NORMAL
	fi" >> /etc/bash.bashrc
	if [ $? != 0 ]; then
	    echo -e $ROUGE "Failed to add /etc/bash.bashrc stuff ! res=$?"  $NORMAL
	    exit 1
	fi
	
	#configuration du prompt
	if [ $IS_HOST == "true" ]; then
		cd /home/ard
		mv .bashrc .bashrc.debian
		wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Dependance/Ard/.bashrc
		dos2unix .bashrc
	else
		#workaround pour compenser le fait que sudo ne soit pas present sans faire d'erreur
		echo "alias sudo=''" >> /root/.bashrc
	fi
	
	
	#installation des scripts utilitaires Ard
	cecho yellow "Installation des script utilitaires d'ARD"
	#download de l'archive
	cd /opt
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Ard/color.sh
	dos2unix color.sh
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Ard/env.sh
	dos2unix env.sh
	
	#configuration des droits sur le scheduler RT
	echo "ard hard rtprio 90" >> /etc/security/limits.conf 
	echo "ulimit -r 90" >> /opt/env.sh

	#droits sudo pour l'ihm
	echo "www-data	ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
	
}

###
# TODO tester que l'image n'existe pas deja
function getting_medias
{
	#récupération des images ARD
	cecho yellow "Récupération des images ..."
	
	#recuperation du logo de boot et de l'ecran de fond
	if [ $IS_HOST == "true" ]; then
		IMG_PATH="/home/ard/Pictures"
	else
		IMG_PATH="/root"
	fi
	cd $IMG_PATH
	wget ftp://ard_user:robotik@88.191.124.77/0%20-%20Docs%20de%20ref/0%20-%20Medias/Logo/Logo-ARD_800_600_fonce.jpg

	echo "
	#wallpaper pour ARD
	GRUB_BACKGROUND=\"$IMG_PATH/Logo-ARD_800_600_fonce.jpg\"
	" >> /etc/default/grub
	if [ $IS_HOST == "true" ]; then
		wget ftp://ard_user:robotik@88.191.124.77/0%20-%20Docs%20de%20ref/0%20-%20Medias/Wallpaper/wallpaper_vm9.jpg
	fi
}

###
# 
function install_ros
{
	#installation de ROS nature (on ne l'installe pas sur la cible c'est trop gros)
	if [ $IS_HOST == "true" ]; then
		cecho yellow "Installation de la distrubtion ROS d'ARD"
	#download de l'archive
		cd /tmp
		wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/ROS/$ROS_VERSION
	
	#extraction
		cd /opt
		tar -xf /tmp/$ROS_VERSION
		ln -sf /opt/ros-$ROS_DISTRIBUTION_update /opt/ros
		rm /tmp/$ROS_VERSION -f
	fi
	
	#ROS distribution ARD, pour tout le monde
	cecho yellow "Installation des addons ROS d'ARD"
	#download de l'archive
	cd /tmp
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/ROS/$ROS_ADDONS_VERSION
	
	#extraction
	tar -xf /tmp/$ROS_ADDONS_VERSION
	ln -sf /opt/ros_addons-$ROS_DISTRIBUTION /opt/ros_addons
	rm /tmp/$ROS_ADDONS_VERSION -f
}

###
#
function install_eclipse
{
	#installation d'eclipse
	if [ $IS_HOST == "true" ]; then
		cecho yellow "Installation d'eclipse $ECLIPSE_VERSION ..."
		echo "#!/bin/bash
		. /opt/env.sh
		/usr/bin/eclipse/eclipse">/usr/bin/eclipse.sh

		chmod 755 /usr/bin/eclipse.sh
		
		cd /tmp
		wget ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Eclipse/$ECLIPSE_VERSION
		
		#clean previous eclipse installation
		rm /usr/bin/eclipse -Rf
		
		cd /usr/bin
		tar -xf /tmp/$ECLIPSE_VERSION
		
		rm /tmp/$ECLIPSE_VERSION
	fi
	
	#recuperation d'une version d'ard
	cd /opt
	svn co svn://88.191.124.77/ARP/trunk ard --username hudson --password robotik --non-interactive --quiet
}

###
# use this function at the end to do a bit of cleaning
function post_install_cleaning
{
	#Correction des droits
	chown ard:ard /opt /home/ard/* /home/ard/.* -R

	#purge du système
	cecho yellow "Purge du système pour gagner de la place"
	#purge des locales
	localepurge

	#suppression du cache apt
	apt-get clean

	if [ $IS_HOST == "false" ]; then
			#suppression de la documentation
		rm -rf /usr/share/doc/
		rm -rf /usr/share/doc-base/
		dpkg --purge man-db manpages
		rm -rf /usr/share/man/
			#vidange des fichiers temporaires
		rm -rf /tmp/*
	fi

	#suppression des paquets orphelins
	deborphan | xargs apt-get -y remove --purge
}

###
#
function configuring_web_server
{	
	cecho yellow "Configuration d'apache2 ... "
	#configuration de apache
	ln -sf /opt/ard/arp_ihm/script/linux/arp_ihm.apache2 /etc/apache2/sites-available/arp_ihm
	a2dissite default
	a2ensite arp_ihm
	/etc/init.d/apache2 reload
}

###
#
function configuring_compilators
{
	#configuration de ccache et distcc
	mkdir -p /home/ard/ccache
	cd /home/ard/ccache
	ln -sf /usr/bin/ccache gcc
	ln -sf /usr/bin/ccache g++
	ln -sf /usr/bin/ccache c++
}

###
#
function installing_ssh_keys
{
	#récupération des clés ssh
	cecho yellow "Add ssh stuff"
	if [ $IS_HOST == "true" ]; then
		mkdir /home/ard/.ssh -p
		cd /home/ard/.ssh
		wget ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Ard/SSH/id_rsa_vm
		mv id_rsa_vm id_rsa
		chmod go-r id_rsa
	
		wget ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Ard/SSH/id_rsa_vm.pub
		mv id_rsa_vm.pub id_rsa.pub
	
		wget ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Ard/SSH/id_rsa_beta.pub
		cat id_rsa_beta.pub >> authorized_keys
	else
		mkdir /root/.ssh -p
		cd /root/.ssh
		wget ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Ard/SSH/id_rsa_beta
		mv id_rsa_beta id_rsa
		chmod go-r id_rsa
	
		wget ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Ard/SSH/id_rsa_beta.pub
		mv id_rsa_beta.pub id_rsa.pub
	
		wget ftp://ard_user:robotik@$WIXIBOX/34%20-%20Info/Dependance/Ard/SSH/id_rsa_vm.pub
		cat id_rsa_vm.pub >> authorized_keys
	fi
	
	echo "UseDNS no" >> /etc/ssh/sshd_config
}

###
#
function configure_init_scripts
{
	cecho yellow "Configuring init scripts ..."
	#recuperation des scripts de boot :
	cd /sbin
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Ard/init_ro
	
	echo "#roslaunch arp_master stratD.launch > /opt/match.log" > /opt/boot.sh
	chmod +x /opt/boot.sh
	if [ $IS_HOST == "false" ]; then
		cd /etc/init.d
		chmod +x /opt/ard/arp_core/script/linux/ard_autostart.sh 
		chmod +x /opt/ard/arp_core/script/linux/ard_boot_ro.sh 
		chmod +x /opt/ard/arp_hml/script/linux/ard_can.sh 
		chmod +x /opt/ard/arp_ihm/script/linux/ard_ihm_launcher.sh

		ln -sf /opt/ard/arp_core/script/linux/ard_boot_ro.sh
		ln -sf /opt/ard/arp_core/script/linux/ard_autostart.sh
		ln -sf /opt/ard/arp_hml/script/linux/ard_can.sh
		ln -sf /opt/ard/arp_ihm/script/linux/ard_ihm_launcher.sh
		
		insserv ard_can.sh
		insserv ard_autostart.sh
	fi

	#desactivation de fancontrol sur la cible qui galere a booter
	if [ $IS_HOST == "false" ]; then
		insserv fancontrol -r
	fi
	
	#creation des regles udev
	ln -sf /opt/ard/arp_hml/script/linux/udev_ubiquity.rules /etc/udev/rules.d/10-udev_ubiquity.rules

	#ajout des bips de démarrage
	sed -i "s|log_action_msg \"Will now halt\"|#beeps;\r\n\t/usr/bin/beep -f 300 -l 300\r\n\t/usr/bin/beep -f 285 -l 300\r\n\t/usr/bin/beep -f 270 -l 500\r\n\r\n\tlog_action_msg \"Will now halt\"|" /etc/init.d/halt 
	sed -i "s|log_action_msg \"Will now restart\"|\r\n\t#beeps;\r\n\t/usr/bin/beep -f 300 -l 300\r\n\t/usr/bin/beep -f 285 -l 300\r\n\t/usr/bin/beep -f 270 -l 300\r\n\t/usr/bin/beep -f 300 -l 300\r\n\r\n\tlog_action_msg \"Will now restart\"|" /etc/init.d/reboot 

}

###
#
function prepare_next_update
{
	#creation du fichier de version
	echo $ARD_VERSION > /opt/kernel/ard-vm-version
	
	
	#récupération du script d'update
	cd /opt/kernel
	wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Publication/livraison_v$ARD_VERSION/check_update-$ARD_VERSION.0.sh
	dos2unix check_update-$ARD_VERSION.0.sh
	ln -sf check_update-$ARD_VERSION.0.sh check_update.sh
}
