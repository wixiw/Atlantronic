#permet de configurer un noeud can via LSS
# il faut fournir 2 arguments : node ID en hexa et baudrate :
# ....
# 3 = 250k
# ....

rosrun socket_can cansend can1 7E5#04.01.00.00.00.00.00.00
rosrun socket_can cansend can1 7E5#5E.00.00.00.00.00.00.00
rosrun socket_can cansend can1 7E5#11.$1.00.00.00.00.00.00
rosrun socket_can cansend can1 7E5#5E.00.00.00.00.00.00.00
rosrun socket_can cansend can1 7E5#13.00.$2.00.00.00.00.00
rosrun socket_can cansend can1 7E5#17.00.00.00.00.00.00.00
rosrun socket_can cansend can1 7E5#04.00.00.00.00.00.00.00
