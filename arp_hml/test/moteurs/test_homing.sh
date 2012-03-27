sh ubiquity_bootup.sh

#homing
#-------

#choisir falling edge pour l'entree ana
rosrun socket_can cansend can0 321#79.00.00.00.00
rosrun socket_can cansend can0 322#79.00.00.00.00
rosrun socket_can cansend can0 323#79.00.00.00.00

#demander l'arret sur le home switch
rosrun socket_can cansend can0 321#90.01.00.00.00
rosrun socket_can cansend can0 322#90.01.00.00.00
rosrun socket_can cansend can0 323#90.01.00.00.00
#choisir la vitesse et le sens de homing
rosrun socket_can cansend can0 321#78.00.04.00.00
rosrun socket_can cansend can0 322#78.00.04.00.00
rosrun socket_can cansend can0 323#78.00.04.00.00

sleep 1

#go home
rosrun socket_can cansend can0 321#2F.00.00.00.00
rosrun socket_can cansend can0 322#2F.00.00.00.00
rosrun socket_can cansend can0 323#2F.00.00.00.00