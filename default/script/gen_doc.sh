#!/bin/bash

# author BMO
# date : 05/03/2011
#
# this script help generating dowygen and eclipse documentation
#
# USER :
# this script takes 1 argument :
# package_name : the name of the package you want the docs to be generated
#

ARP_PROJECT_NAME=$1

VERT="\\033[1;32m" 
NORMAL="\\033[0;39m" 
ROUGE="\\033[1;31m" 
ROSE="\\033[1;35m" 
BLEU="\\033[1;34m" 
BLANC="\\033[0;02m" 
BLANCLAIR="\\033[1;08m" 
JAUNE="\\033[1;33m" 
CYAN="\\033[1;36m" 

#check inputs
if [ $1 == "" ]
then
	echo -e $ROUGE "[!] you did not provide correct arguments. Please add a package_name" $NORMAL
	echo -e $ROUGE "[!] example : sh gen_doc.sh arp_core" $NORMAL
fi

echo -e $BLEU "[+] deleting old documention in $ARP_PROJECT_NAME package..." $NORMAL
rm -rf doc/html
rm -rf doc/latex

echo -e $BLEU "[+] generating $ARP_PROJECT_NAME documentation ..." $NORMAL
doxygen doc/Doxyfile.sh
if [ $? != 0 ]
then
	echo -e $ROUGE "[!] Doxygen fail to generate the documentation" $NORMAL
fi

echo -e $BLEU "[+] integration into Eclipse Help Contents ..." $NORMAL
mkdir -p /usr/bin/eclipse/plugins/org.ard.$ARP_PROJECT_NAME
ln -sf `rospack find $1`/doc/html /usr/bin/eclipse/plugins/org.ard.$ARP_PROJECT_NAME/html

echo -e $VERT "[.] doc de Core intégrée dans l'aide d'eclipse. Dispo dans Help/Help Contents (redémarrage d'eclipse éventuellement nécessaire)" $NORMAL
