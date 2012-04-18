#!/bin/bash -ex
# auteur : WLA
# data : 18/04/2012
# version 10.0

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
configure_boot
create_SYA
configure_networks
configure_version_systems
configure_user_profiles
configuring_web_server
configuring_compilators
installing_ssh_keys
configure_init_scripts

#suppression du probleme avec exim4
sed -i "s|dc_local_interfaces='127.0.0.1 ; ::1'|dc_local_interfaces='127.0.0.1'|" /etc/exim4/update-exim4.conf
rm /var/log/exim4/paniclog

#########
# PHASE 2 : heavy dependencies
#########

install_kernel
install_osdeps
install_xenomai
install_ros
install_eclipse


#########
# PHASE 3 : cleaning 
#########

prepare_next_update
post_install_cleaning


cecho green "Succeed !"
df / -h
if [ $IS_HOST == "false" ]; then
	cecho bluegreen "Don't forget to do an 'install all' from eclipse to complete the ros folder."
fi
