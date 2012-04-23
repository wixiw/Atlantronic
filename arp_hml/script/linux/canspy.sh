if [ $CAN_FLAVOR == "gnulinux" ]
then
	rosrun socket_can candump -t a -a -c -e any,0:0,#FFFFFFFF
	exit
fi

if [ $CAN_FLAVOR == "xenomai" ]
then
	rtcanrecv rtcan0 -vT -e=0xFFFFFFFF
	exit
fi

echo "Cannot not determine if you are using orocos or xenomai. Please set CAN_FLAVOR".
