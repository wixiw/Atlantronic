#!/bin/bash

sh ubiquity_bootup.sh

limit_torque()
{
	rosrun socket_can cansend can0 321#80.00.20.00.00
	rosrun socket_can cansend can0 322#80.00.20.00.00
	rosrun socket_can cansend can0 323#80.00.20.00.00
	rosrun socket_can cansend can0 331#80.00.20.00.00
	rosrun socket_can cansend can0 332#80.00.20.00.00
	rosrun socket_can cansend can0 333#80.00.20.00.00

	rosrun socket_can cansend can0 321#81.00.20.00.00
	rosrun socket_can cansend can0 322#81.00.20.00.00
	rosrun socket_can cansend can0 323#81.00.20.00.00
	rosrun socket_can cansend can0 331#81.00.20.00.00
	rosrun socket_can cansend can0 332#81.00.20.00.00
	rosrun socket_can cansend can0 333#81.00.20.00.00
	sleep 1
}

turn()
{
	#loadd relative target
	rosrun socket_can cansend can0 321#B6.00.B8.00.00
	rosrun socket_can cansend can0 322#B6.00.B8.00.00
	rosrun socket_can cansend can0 323#B6.00.B8.00.00
	
	#disable traction
	rosrun socket_can cansend can0 331#08.00.00.00.00
	rosrun socket_can cansend can0 332#08.00.00.00.00
	rosrun socket_can cansend can0 333#08.00.00.00.00
	
	#go !
	rosrun socket_can cansend can0 321#3C.00.00.00.00
	rosrun socket_can cansend can0 322#3C.00.00.00.00
	rosrun socket_can cansend can0 323#3C.00.00.00.00
	sleep 1
	
	rosrun socket_can cansend can0 331#0F.00.00.00.00
	rosrun socket_can cansend can0 332#0F.00.00.00.00
	rosrun socket_can cansend can0 333#0F.00.00.00.00
}

forward()
{
	rosrun socket_can cansend can0 331#93.00.04.00.00
	rosrun socket_can cansend can0 332#93.00.04.00.00
	rosrun socket_can cansend can0 333#93.00.04.00.00
	sleep 2
	rosrun socket_can cansend can0 331#93.00.00.00.00
	rosrun socket_can cansend can0 332#93.00.00.00.00
	rosrun socket_can cansend can0 333#93.00.00.00.00
}

limit_torque

forward
turn

forward
turn

forward
turn

forward
turn
