#!/bin/bash -e
# auteur : WLA
# data : 30/04/2012
# version 10.0

# This script allows you change the network config when coding alone at home or in team for the contest.

###############################################################################################################

VM_NAME=ard-host

###############################################################################################################
#source env datas
. /opt/color.sh
. /opt/env.sh

#if DEBUG env variable is set to something, use x option whihc prints each executed line
[ -n "$DEBUG" ] && set -x
set -e

#check root user
if [[ $UID -ne 0 ]]; then
	cecho red "$0 must be run as root"
	exit 1
fi 

function help
{
	cecho red "[!] you did not provide correct arguments. Please enter a network_config and team_member"
	cecho blue "examples :"
	cecho blue "bash switch_network.sh ferte moulineau"
	cecho blue "bash switch_network.sh home boris"
}

#check args
if [[ $# != 2 ]] ; then
	cecho red "[!] I need 2 arguments"
	help
	return 0
else
	case "$1" in
		ferte|home)
		network_config=$1
			;;
		*)
			cecho red "[!] First arg is either 'home' or 'ferte'
			help
			return 0
			;;
	esac
	case "$2" in
		boris|moulineau|willy)
		team_member=$2
			;;
		*)
			cecho red "[!] Second arg is either 'boris', 'moulineau' or 'willy'
			help
			return 0
			;;
	esac
fi

###############################################################################################################


case "$network_config" in
	
		ferte)
			sed -i "s/export DISTCC_HOSTS=\"localhost\"/export DISTCC_HOSTS=\"willy boris localhost moulineau\"/" /opt/env.sh
			sed -i "s/export ROS_PARALLEL_JOBS=-j4/export ROS_PARALLEL_JOBS=-j10/" /opt/env.sh
			echo $team_member > /etc/hostname
			echo "
127.0.0.1	localhost
127.0.1.1	$team_member.team-ard.com	$team_member
127.0.0.1       vm      arp_ihm
10.0.0.29       alpha.team-ard.com      alpha
10.0.0.41        beta.team-ard.com       beta
10.0.0.253   acksys   acksys
88.191.124.77	    wixibox

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
" > /etc/hosts
			;;
			
			
			
			
			
			
		home*)
			sed -i "s/export DISTCC_HOSTS=\"willy boris localhost moulineau\"/export DISTCC_HOSTS=\"localhost\"/" /opt/env.sh
			sed -i "s/export ROS_PARALLEL_JOBS=-j10/export ROS_PARALLEL_JOBS=-j4/" /opt/env.sh
			echo $VM_NAME > /etc/hostname
			echo "
127.0.0.1	localhost
127.0.1.1	$VM_NAME.team-ard.com	$VM_NAME
127.0.0.1       vm      arp_ihm
10.0.0.29       alpha.team-ard.com      alpha
10.0.0.41        beta.team-ard.com       beta
10.0.0.253   acksys   acksys
88.191.124.77	    wixibox

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
" > /etc/hosts
			;;
			
			
	esac
