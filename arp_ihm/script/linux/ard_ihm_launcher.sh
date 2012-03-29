#! /bin/bash
### BEGIN INIT INFO
# Provides:          ard_ihm_launcher
# Required-Start:    $remote_fs $syslog ard_can
# Required-Stop:     $remote_fs $syslog ard_can
# Default-Start:     
# Default-Stop:      0 1 6
# Short-Description: roslaunch in background
# Description:      Use this script from IHM to launch a program in background
### END INIT INFO

# Author:A.R.D. <contact@team-ard.com>

# Do NOT "set -e"

# PATH should only include /usr/* if it runs after the mountnfs.sh script
PATH=/sbin:/usr/sbin:/bin:/usr/bin
DESC="Use this script from IHM to launch a program in background"
NAME="ard_ihm_launcher"
DAEMON="/opt/ros/ros/bin/roslaunch"
DAEMON_ARGS=""
PIDFILE="/var/run/ard/$NAME.pid"
SCRIPTNAME="/etc/init.d/$NAME.sh"


# Exit if the environnement variables are not present
if [ -x "/opt/env.sh" ] ; then
	echo "ERROR ($NAME) : The /opt/env.sh script is not present"
	exit 0
else
	. /opt/env.sh	
fi


# Exit if the package is not installed
if [ ! -x "$DAEMON" ] ; then
	cecho red "$DAEMON is missing on the system or it doesn't have execution rigths"
	exit 0
fi

# Read configuration variable file if it is present
[ -r /etc/default/$NAME ] && . /etc/default/$NAME

# Load the VERBOSE setting and other rcS variables
. /lib/init/vars.sh

# Define LSB log_* functions.
# Depend on lsb-base (>= 3.2-14) to ensure that this file is present
# and status_of_proc is working.
. /lib/lsb/init-functions

#check if the user is root
if [[ $UID -ne 0 ]]; then
	SUDO="sudo"
else
	SUDO=""
fi

#
# Function that starts the daemon/service
#
do_start()
{
	# Return
	#   0 if daemon has been started
	#   1 if daemon was already running
	#   2 if daemon could not be started
	$SUDO  start-stop-daemon --start --quiet --pidfile $PIDFILE --exec $DAEMON --test > /dev/null \
		|| return 1
		
	echo ""
		
	if [ -f $PIDFILE ] ; then
		cecho yellow "A daemon is already running"
		return 1
	fi

	if [[ $1 == "" ]]; then
		cecho red "You must provide a package name and a launch file !"
		cecho blue "Try it with $SCRIPTNAME arp_hml truc.launch for example."
		return 2
	else if [[ `$SUDO rospack find $1` == "" ]]; then
			cecho red "You provided $1 as a package name but rospack failed to find it"
			return 2
		fi
	fi
	
	
	if [[ $2 == "" ]]; then
		cecho red "You must provide a launch file !"
		return 2
	else
		#S=start m=create pid file b=go to background v=verbose
		start-stop-daemon -Smbv -x $SUDO $DAEMON $1 $2 -p $PIDFILE 
		
		cecho yellow "Launching a deamon on IHM request with param : $1 $2"
	fi
}

#
# Function that stops the daemon/service
#
do_stop()
{
	echo ""
	if [ -f $PIDFILE ] ; then	
		cecho "yellow" "Killing the previously launched program !"
		$SUDO  pkill -KILL -P `cat $PIDFILE` 
		$SUDO  rm -f $PIDFILE
		return 0
	else
		cecho "yellow" "No program is running for now..."
	fi
}


case "$1" in
  start)
	[ "$VERBOSE" != no ] && log_daemon_msg "Starting $DESC" "$NAME"
	do_start $2 $3
	case "$?" in
		0|1) [ "$VERBOSE" != no ] && log_end_msg 0 ;;
		2) [ "$VERBOSE" != no ] && log_end_msg 1 ;;
	esac
	;;
  stop)
	[ "$VERBOSE" != no ] && log_daemon_msg "Stopping $DESC" "$NAME"
	do_stop $2 $3
	case "$?" in
		0|1) [ "$VERBOSE" != no ] && log_end_msg 0 ;;
		2) [ "$VERBOSE" != no ] && log_end_msg 1 ;;
	esac
	;;
  status)
       status_of_proc "$DAEMON" "$NAME" && exit 0 || exit $?
    ;;

  *)
	echo "Usage: $SCRIPTNAME {start|stop|status}" >&2
	exit 3
	;;
esac

:
