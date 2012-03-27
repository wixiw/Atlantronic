#permet de configurer un noeud can via LSS
# il faut fournir 3 arguments : 
# _ le bus can sur lequel est branche le moteur
# _ le node ID en hexa 
# _ baudrate à l'aide d'un code :
# 00 = 1000k
# 01 = 800k
# 03 = 250k
# 04 = 125k
# 05 = reserved
# 06 = 50k
# 07 = 20k
# 08 = 10k
#

VERT="\\033[1;32m" 
NORMAL="\\033[0;39m" 
ROUGE="\\033[1;31m" 
ROSE="\\033[1;35m" 
BLEU="\\033[1;34m" 
BLANC="\\033[0;02m" 
BLANCLAIR="\\033[1;08m" 
JAUNE="\\033[1;33m" 
CYAN="\\033[1;36m" 

can=$1
node_id=$2
baudrate_code=$3

#check inputs
if [ $# != 3 ]
then
	echo -e $ROUGE "[!] you did not provide correct arguments. Please enter a can bus, a node_id in hexa and an baudrate_code (00=1M;1=800k;02=500k;03=250k;...)" $NORMAL
echo -e $ROUGE "example noeud 0x33 à 250k : sh lss_configure.sh can0 33 03" $NORMAL
	exit 0
fi


rosrun socket_can cansend $can 7E5#04.01.00.00.00.00.00.00
rosrun socket_can cansend $can 7E5#5E.00.00.00.00.00.00.00
rosrun socket_can cansend $can 7E5#11.$node_id.00.00.00.00.00.00
rosrun socket_can cansend $can 7E5#5E.00.00.00.00.00.00.00
rosrun socket_can cansend $can 7E5#13.00.$baudrate_code.00.00.00.00.00
rosrun socket_can cansend $can 7E5#17.00.00.00.00.00.00.00
rosrun socket_can cansend $can 7E5#04.00.00.00.00.00.00.00
