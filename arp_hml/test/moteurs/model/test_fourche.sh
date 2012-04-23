rosrun socket_can cansend can1 000#81.00
rosrun socket_can cansend can1 000#01.00

rosrun socket_can cansend can1 323#FD.FF.FF.FF.FF
rosrun socket_can cansend can1 323#0F.00.00.00.00

#FAULHABER MODE
###############

#demande au moteur de répondre immédiatement au faulhaber commands
rosrun socket_can cansend can1 623#2F.01.18.02.FF.00.00.00

#speed 
#-------

rosrun socket_can cansend can1 323#93.00.04.00.00
#arret
rosrun socket_can cansend can1 323#93.00.00.00.00


#position
#--------

#set 0 home position
rosrun socket_can cansend can1 323#B8.00.00.00.00
#load absolute target
rosrun socket_can cansend can1 323#B4.00.00.00.00
#load relative target
rosrun socket_can cansend can1 323#B6.00.00.01.00
#go !
rosrun socket_can cansend can1 323#3C.00.00.00.00


#homing
#-------

#choisir falling edge pour l'entree ana
rosrun socket_can cansend can1 323#79.00.00.00.00
#demander l'arret sur le home switch
rosrun socket_can cansend can1 323#90.01.00.00.00
#choisir la vitesse et le sens de homing
rosrun socket_can cansend can1 323#78.00.04.00.00
#go home
rosrun socket_can cansend can1 323#2F.00.00.00.00

#DS402
#############

#send control word (fault reset, shutdown, switch on, enable)
rosrun socket_can cansend can1 623#2B.40.60.00.80.00.00.00
rosrun socket_can cansend can1 623#2B.40.60.00.06.00.00.00
rosrun socket_can cansend can1 623#2B.40.60.00.07.00.00.00
rosrun socket_can cansend can1 623#2B.40.60.00.0F.00.00.00

#position
#--------

#switching to pos mode
rosrun socket_can cansend can1 323#FD.01.00.00.00
#target
rosrun socket_can cansend can1 623#23.7A.60.00.00.00.01.00
#go !
rosrun socket_can cansend can1 623#2B.40.60.00.3F.00.00.00

#homing
#----------

#definir la vitesse de homing à 0x0400
rosrun socket_can cansend can1 623#23.99.60.02.00.04.00.00
#homing method DS402 numero 19=0x13
rosrun socket_can cansend can1 623#2F.98.60.00.13.00.00.00
#choisir l'entrée analogique comme home switch
rosrun socket_can cansend can1 623#2F.10.23.03.01.00.00.00


