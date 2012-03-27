sh ubiquity_bootup.sh

rosrun socket_can cansend can0 321#93.FF.04.00.00
rosrun socket_can cansend can0 322#93.FF.04.00.00
rosrun socket_can cansend can0 323#93.FF.04.00.00
rosrun socket_can cansend can0 331#93.FF.04.00.00
rosrun socket_can cansend can0 332#93.FF.04.00.00
rosrun socket_can cansend can0 333#93.FF.04.00.00

sleep 10

rosrun socket_can cansend can0 321#93.00.00.00.00
rosrun socket_can cansend can0 322#93.00.00.00.00
rosrun socket_can cansend can0 323#93.00.00.00.00
rosrun socket_can cansend can0 331#93.00.00.00.00
rosrun socket_can cansend can0 332#93.00.00.00.00
rosrun socket_can cansend can0 333#93.00.00.00.00
