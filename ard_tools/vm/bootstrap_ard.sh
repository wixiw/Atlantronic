#!/bin/bash -ex
# auteur : WLA
# data : 18/04/2012
# version 9.3

###############################################################################################################

#if DEBUG env variable is set to something, use x option whihc prints each executed line
[ -n "$DEBUG" ] && set -x

#check root user
if [[ $UID -ne 0 ]]; then
	echo -e $ROUGE "$0 must be run as root" $NORMAL
	exit 1
fi

#source env datas
. ./color.sh
. ./bootstrap_config.sh
. ./bootstrap_functions.sh

#check IP connexion
check_wixibox

#########
# PHASE 1 : system configuration
#########

create_folders
populated_rc.local
configure_apt
apt-get update -f
configure_grub
create_SYA
configure_networks
configure_version_systems
configure_user_profiles
configuring_web_server
configuring_compilators
installing_ssh_keys
configure_init_scripts

#suppression du checkdisk (seulement sur les targets)
if [ $IS_HOST == "false" ]; then
	echo -e $JAUNE "Suppression du checkdisk au boot..." $NORMAL
	sed -i "s/\/               ext2    errors=remount-ro 0       1/\/               ext2    errors=remount-ro 0       0/" /etc/fstab
	if [ $? != 0 ]; then
	    echo -e $ROUGE "Failed to desactivate checkdisk in /etc/fstab ! res=$?"  $NORMAL
	    exit 1
	fi
fi

#correction de la ligne cdrom
echo "/dev/hdc	/media/cdrom0	udf,iso9660	user,noauto	0	0" >> /etc/fstab


#creation du repertoire de conf pour ce qui est lie a l'OS
echo -e $JAUNE "Creating ARD OS conf folder" $NORMAL
mkdir /opt/conf -p

echo "You *may* create empty files here to configure the way ARD's OS load :
_ /opt/conf/MATCH : autostart main binairies for matches. 
_ /opt/conf/RO : load a read only filesystem to protect the flash

You *must* have a /opt/conf/OS containing either 'gnulinux' or 'xenomai' to select Orocos flavor." > /opt/conf/README
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to create /opt/conf/README"  $NORMAL
    exit 1
fi
echo "gnulinux" > /opt/conf/OS
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to create /opt/conf/OS"  $NORMAL
    exit 1
fi


#installation des scripts utilitaires Ard
echo -e $JAUNE "Installation des script utilitaires d'ARD" $NORMAL
#download de l'archive
cd /opt
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Ard/color.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to download color.sh from the FTP ! res=$?"  $NORMAL
    exit 1
fi
dos2unix color.sh
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Ard/env.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to download color.sh from the FTP ! res=$?"  $NORMAL
    exit 1
fi
dos2unix env.sh


#########
# PHASE 2 : heavy dependencies
#########

install_kernel
install_osdeps
install_xenomai
install_ros
install_eclipse

#recuperation d'une version d'ard
cd /opt
svn co svn://88.191.124.77/ARP/trunk ard --username hudson --password robotik --non-interactive






#########
# PHASE 3 : cleaning 
#########

#creation du fichier de version
echo $ARD_VERSION > /opt/kernel/ard-vm-version
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to define vm version in /opt/kernel/ard-vm-version"  $NORMAL
    exit 1
fi

#récupération du script d'update
cd /opt/kernel
wget ftp://ard_user:robotik@88.191.124.77/34%20-%20Info/Publication/livraison_v$ARD_VERSION/check_update-$ARD_VERSION.0.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to download check_update script."  $NORMAL
    exit 1
fi
dos2unix check_update-$ARD_VERSION.0.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to dos2unix /opt/kernel/check_update.sh"  $NORMAL
    exit 1
fi
ln -sf check_update-$ARD_VERSION.0.sh check_update.sh
if [ $? != 0 ]; then
    echo -e $ROUGE "Failed to create symlink /opt/kernel/check_update.sh"  $NORMAL
    exit 1
fi


post_install_cleaning


echo -e $VERT "Succeed !"
df / -h
echo -e $NORMAL
