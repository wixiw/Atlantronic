#!/bin/bash -ex
# auteur : WLA
# data : 18/04/2012
# version 10.0

###############################################################################################################

#if DEBUG env variable is set to something, use x option whihc prints each executed line
[ -n "$DEBUG" ] && set -x
set -e

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

mount_tmpOnRam

#recuperation d'une version d'ard
cecho yellow "Get ard repository ..."
cecho blue "it will hang a bit, it's normal (quiet option), it works in background"
cd /opt
export wixibox=88.191.124.77
svn co svn://wixibox/ARP/trunk ard --username hudson --password robotik --non-interactive --quiet

create_folders
configure_apt
install_osdeps #on ne peut pas attendre la phase 2 puisque certains fichiers de config sont à revoir
populated_rc.local
configure_boot
create_SYA
configure_network
configure_version_systems
configure_user_profiles
configuring_web_server
configuring_compilators
installing_ssh_keys
configure_init_scripts
getting_medias

#suppression du probleme avec exim4
sed -i "s|dc_local_interfaces='127.0.0.1 ; ::1'|dc_local_interfaces='127.0.0.1'|" /etc/exim4/update-exim4.conf.conf
rm /var/log/exim4/paniclog -f

#########
# PHASE 2 : heavy dependencies
#########

install_kernel
install_xenomai
install_ros
install_eclipse

#correction d'un bug dans le projet eclipse
cd /opt/ard
svn switch --relocate svn://88.191.124.77 svn://wixibox

#########
# PHASE 3 : cleaning 
#########

cecho yellow "Updating search file cache..."
updatedb

prepare_next_update
post_install_cleaning


cecho green "Succeed !"
df / -h
if [ $IS_HOST == "false" ]; then
	cecho blue "Don't forget to do an 'install all' from eclipse to complete the ros folder."
fi
