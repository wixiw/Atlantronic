if [ $OROCOS_TARGET == "gnulinux" ]
then
	rosrun socket_can candump -t a -a -c -e any,0:0,#FFFFFFFF
	exit
fi

if [ $OROCOS_TARGET == "xenomai" ]
then
	rosrun socket_can candump -t a -a -c -e any,0:0,#FFFFFFFF
	exit
fi

echo "Cannot not determine if you are using orocos or xenomai. Please set OROCOS_TARGET".
