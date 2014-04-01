#!/bin/bash
IS_HOST="true"
. /opt/color.sh

#define target dependant variables
if [ $HOSTNAME == "alpha" ]; then
        cecho blue "[!] you are on the alpha target !"
	IS_HOST="false"
else 
	if [ $HOSTNAME == "beta" ]; then
		cecho blue "[!] you are on the beta target !"
		IS_HOST="false"
	else
		cecho blue "[!] you are on the VM target !"
	fi
fi

#verification des versions
VM_CURRENT_VERSION=`cat /opt/kernel/ard-vm-version`
if [ $VM_CURRENT_VERSION == "10.1" ]; then
	cecho green "VM already in version 10.1, which is the last version"
	exit 0
fi
if [ $VM_CURRENT_VERSION != "10.0" ]; then
	cecho red "VM is in an unexpected version $VM_CURRENT_VERSION"
	exit 1
fi

################################




# hack hack hack ...








################################


#recuperation des notes de version
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Publication/livraison_v109.0/patch_10.1/release-note-10.1.txt


#succes
cecho green "SUCCEED !!!"
cecho yellow `cat release-note-10.1.txt`
cecho green "please reboot and reinstall VM additions and then retry ard-update to check next update"
echo "10.1" > /opt/kernel/ard-vm-version 
