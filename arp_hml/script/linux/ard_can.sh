#! /bin/bash
### BEGIN INIT INFO
# Provides:          ard_can
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Launch can drivers
# Description:       This script launch can drivers. Depending os the /opt/conf/OS value (gnulinux or xenomai) it launches socket_can gnulinux drivers or xenomai rtdm drivers
### END INIT INFO

# Author: Foo Bar <contact@team-ard.com>


# Do NOT "set -e"

# PATH should only include /usr/* if it runs after the mountnfs.sh script
PATH=/sbin:/usr/sbin:/bin:/usr/bin
DESC="can drivers"
NAME="ard_can"
PIDFILE=/var/run/ard_can.pid
SCRIPTNAME=/etc/init.d/ard_can
BAUDRATE=1000000
#notice that we can only set one IRQ for both can, this is an hardware limitation
CAN_OPTS="irq=10,10 mem=0xD0000,0xD0200 ocr=0x5e,0x5e cdr=0,0"

# Read configuration variable file if it is present
[ -r /etc/default/$NAME ] && . /etc/default/$NAME

# Load the VERBOSE setting and other rcS variables
. /lib/init/vars.sh

# Define LSB log_* functions.
# Depend on lsb-base (>= 3.2-14) to ensure that this file is present
# and status_of_proc is working.
. /lib/lsb/init-functions

. /opt/color.sh
. /opt/ard/env.sh

#
# Function that starts the daemon/service
#
do_start()
{
	if [[ $CAN_FLAVOR == "xenomai" ]] 
	then
		cecho yellow "Starting Xenomai can ..."
		modprobe xeno_can_mem $CAN_OPTS
		rtcanconfig rtcan0 -b $BAUDRATE -v start
		rtcanconfig rtcan1 -b $BAUDRATE -v start
	else
		cecho yellow "Starting Gnulinux can ..."
		modprobe sja1000_isa $CAN_OPTS
	
		#chargement de l'interface dans ifconfig
		ip link set can0 type can bitrate $BAUDRATE restart-ms 1000
		ip link set can1 type can bitrate $BAUDRATE restart-ms 1000
	
		#demarrage de l'interface
		ifconfig can0 up
		ifconfig can1 up
	fi
}

#
# Function that stops the daemon/service
#
do_stop()
{
	if [[ $CAN_FLAVOR  == "xenomai" ]] 
	then
		cecho yellow "Stopping Xenomai can ..."
		rmmod xeno_can_mem xeno_can_sja1000 xeno_can
	else
		cecho yellow "Stoppping Gnulinux can ..."
		rmmod sja1000_isa sja1000 can_dev
	fi
}

#
# Function that sends a SIGHUP to the daemon/service
#
do_reload() {
	echo "unimplemented"
}

case "$1" in
  start)
	[ "$VERBOSE" != no ] && log_daemon_msg "Starting $DESC" "$NAME"
	do_start
	case "$?" in
		0|1) [ "$VERBOSE" != no ] && log_end_msg 0 ;;
		2) [ "$VERBOSE" != no ] && log_end_msg 1 ;;
	esac
	;;
  stop)
	[ "$VERBOSE" != no ] && log_daemon_msg "Stopping $DESC" "$NAME"
	do_stop
	case "$?" in
		0|1) [ "$VERBOSE" != no ] && log_end_msg 0 ;;
		2) [ "$VERBOSE" != no ] && log_end_msg 1 ;;
	esac
	;;
  status)
       status_of_proc "$DAEMON" "$NAME" && exit 0 || exit $?
       ;;
  #reload|force-reload)
	#
	# If do_reload() is not implemented then leave this commented out
	# and leave 'force-reload' as an alias for 'restart'.
	#
	#log_daemon_msg "Reloading $DESC" "$NAME"
	#do_reload
	#log_end_msg $?
	#;;
  restart|force-reload)
	#
	# If the "reload" option is implemented then remove the
	# 'force-reload' alias
	#
	log_daemon_msg "Restarting $DESC" "$NAME"
	do_stop
	case "$?" in
	  0|1)
		do_start
		case "$?" in
			0) log_end_msg 0 ;;
			1) log_end_msg 1 ;; # Old process is still running
			*) log_end_msg 1 ;; # Failed to start
		esac
		;;
	  *)
	  	# Failed to stop
		log_end_msg 1
		;;
	esac
	;;
  *)
	#echo "Usage: $SCRIPTNAME {start|stop|restart|reload|force-reload}" >&2
	echo "Usage: $SCRIPTNAME {start|stop|status|restart|force-reload}" >&2
	exit 3
	;;
esac

:
