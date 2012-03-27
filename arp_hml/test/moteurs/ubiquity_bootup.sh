#use this script to initilize the motors

rosrun socket_can cansend can0 000#81.00

sleep 2

rosrun socket_can cansend can0 000#01.00

sleep 2

rosrun socket_can cansend can0 321#FD.FF.FF.FF.FF
rosrun socket_can cansend can0 322#FD.FF.FF.FF.FF
rosrun socket_can cansend can0 323#FD.FF.FF.FF.FF
rosrun socket_can cansend can0 331#FD.FF.FF.FF.FF
rosrun socket_can cansend can0 332#FD.FF.FF.FF.FF
rosrun socket_can cansend can0 333#FD.FF.FF.FF.FF

sleep 2

rosrun socket_can cansend can0 321#0F.00.00.00.00
rosrun socket_can cansend can0 322#0F.00.00.00.00
rosrun socket_can cansend can0 323#0F.00.00.00.00
rosrun socket_can cansend can0 331#0F.00.00.00.00
rosrun socket_can cansend can0 332#0F.00.00.00.00
rosrun socket_can cansend can0 333#0F.00.00.00.00

sleep 2

#FAULHABER MODE
###############

#demande au moteur de répondre immédiatement au faulhaber commands
rosrun socket_can cansend can0 621#2F.01.18.02.FF.00.00.00
rosrun socket_can cansend can0 622#2F.01.18.02.FF.00.00.00
rosrun socket_can cansend can0 623#2F.01.18.02.FF.00.00.00

sleep 2