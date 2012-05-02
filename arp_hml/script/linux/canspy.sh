if [ $CAN_FLAVOR == "gnulinux" ]
then
	rosrun socket_can candump -t a -a -c -e any,0:0,#FFFFFFFF
	exit
fi

if [ $CAN_FLAVOR == "xenomai" ]
then
	rtcanrecv -vT -e=0xFFFFFFFF
	exit
fi

echo "Cannot not determine if you are using orocos or xenomai. Please set CAN_FLAVOR".


#
# filtrer les messages NMT :
# -f0x0;0xFFFFFFFFFF -f0x705:0xFFFFFFFF -f0x706:0xFFFFFFFF -f0x721:0xFFFFFFFF -f0x722:0xFFFFFFFF -f0x723:0xFFFFFFFF -f0x731:0xFFFFFFFF -f0x732:0xFFFFFFFF -f0x733:0xFFFFFFFF
#