rosrun socket_can cansend can1 000#81.00
rosrun socket_can cansend can1 000#01.00

rosrun socket_can cansend can1 321#FD.FF.FF.FF.FF
rosrun socket_can cansend can1 321#0F.00.00.00.00

#FAULHABER MODE
###############

#demande au moteur de répondre immédiatement au faulhaber commands
rosrun socket_can cansend can1 621#2F.01.18.02.FF.00.00.00
#set speed 
rosrun socket_can cansend can1 321#93.00.04.00.00
#arret
rosrun socket_can cansend can1 321#93.00.00.00.00

#homing
#-------

#choisir falling edge pour l'entree ana
rosrun socket_can cansend can1 321#79.00.00.00.00
#demander l'arret sur le home switch
rosrun socket_can cansend can1 321#90.01.00.00.00
#choisir la vitesse et le sens de homing
rosrun socket_can cansend can1 321#78.00.04.00.00
#go home
rosrun socket_can cansend can1 321#2F.00.00.00.00

#DS402
#############

#homing
#----------

#definir la vitesse de homing à 0x0400
rosrun socket_can cansend can1 621#23.99.60.02.00.04.00.00
#homing method DS402 numero 19=0x13
rosrun socket_can cansend can1 621#2F.98.60.00.13.00.00.00
#choisir l'entrée analogique comme home switch
rosrun socket_can cansend can1 621#2F.10.23.03.01.00.00.00


