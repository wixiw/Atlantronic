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
if [ $VM_CURRENT_VERSION == "9.2" ]; then
	cecho green "VM already in version 9.2, which is the last version"
	exit 0
fi
if [ $VM_CURRENT_VERSION != "9.1" ]; then
	cecho red "VM is in an unexpected version $VM_CURRENT_VERSION"
	exit 1
fi

#installation de dependances os pour analyse de courbes et montage filesystem distant
cecho yellow "Installation de dépendances système"
if [ $IS_HOST == "true" ]; then
	apt-get install -y kst sshfs libomniorb4-dev libomnithread3-dev omniidl omniidl4 omniorb omniorb-nameserver omniorb4 omniorb4-nameserver
	if [ $? != 0 ]; then
	    cecho -red "Failed to install OS packages! res=$?"
	    exit 1
	fi
else
	apt-get install -y sshfs libboost-program-options1.42.0 libboost-signals1.42.0 libboost-filesystem1.42.0 libboost-thread1.42.0 libboost-regex1.42.0 liblog4cxx10 liblua5.1-0


	if [ $? != 0 ]; then
	    cecho -red "Failed to install sshfs! res=$?"
	    exit 1
	fi
fi

#configuration des droits sur le scheduler RT
echo "ard hard rtprio 90" >> /etc/security/limits.conf 
echo "ulimit -r 90" >> /opt/env.sh

#droits sudo pour l'ihm
echo "www-data	ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


#liens symboliques de /etc/init.d
cecho yellow "Link des daemons ard"
cd /etc/init.d
chmod +x /opt/ard/arp_core/script/linux/ard_autostart.sh 
chmod +x /opt/ard/arp_core/script/linux/ard_boot_ro.sh 
chmod +x /opt/ard/arp_hml/script/linux/ard_can.sh 
chmod +x /opt/ard/arp_ihm/script/linux/ard_ihm_launcher.sh

ln -sf /opt/ard/arp_core/script/linux/ard_autostart.sh
ln -sf /opt/ard/arp_core/script/linux/ard_boot_ro.sh
ln -sf /opt/ard/arp_hml/script/linux/ard_can.sh
ln -sf /opt/ard/arp_ihm/script/linux/ard_ihm_launcher .sh
if [ $IS_HOST == "false" ]; then
	insserv ard_can.sh
	if [ $? != 0 ]; then
	    cecho -red "Failed to insert ard_can.sh daemon ! res=$?"
	    exit 1
	fi
	insserv ard_autostart.sh
	if [ $? != 0 ]; then
	    cecho -red "Failed to insert ard_autostart.sh daemon ! res=$?"
	    exit 1
	fi
fi
cd /



#configuration ihm locale
cecho yellow "Configuration d'apache2"
a2dissite default
a2ensite arp_ihm
/etc/init.d/apache2 reload
if [ $? != 0 ]; then
    cecho -red "Failed to reload apache2 configuration ! res=$?"
    exit 1
fi

#correction d'un lien de env.sh
sed -i "s|RTTLUA_MODULES=/opt/ros_addons/orocos_toolchain/ocl/lua/modules/?.lua|RTTLUA_MODULES=\`rospack find ocl\`/lua/modules/?.lua|" /opt/env.sh
if [ $? != 0 ]; then
    cecho -red "Failed to correct LUA modules path"
    exit 1
fi

cecho yellow "Les opérations suivantes peuvent être longue, merci d'être patient"

#acquisition du nouveau ROS

cd /opt
cecho yellow "sauvegarde des addons de la distrubtion ROS d'ARD 9.1 dans /opt"
	#sauvegarde du dossier courant
tar -czf ros_addons_prePatch-9.2.tgz ros_addons-electric
if [ $? != 0 ]; then
    cecho -red "Failed to tar current ros_addons-electric dir res=$?"
    exit 1
fi
rm -Rf /opt/ros_addons-electric
cecho yellow "Téléchargemennt des addons de la distrubtion ROS d'ARD 9.2"
	#download de l'archive
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/ROS/ros_addons-electric_ard-9.2_i386.tgz
if [ $? != 0 ]; then
    cecho -red "Failed to download ros_addons-electric_ard-9.2_i386.tgz from the FTP ! res=$?"
    exit 1
fi
cecho yellow "Installation des addons de la distrubtion ROS d'ARD 9.2"
#extraction
tar -xf ros_addons-electric_ard-9.2_i386.tgz
if [ $? != 0 ]; then
    cecho red "Failed to untar ros_addons-electric_ard-9.2_i386.tgz ! res=$?"
    exit 1
fi
rm ros_addons-electric_ard-9.2_i386.tgz -f


cecho yellow "Ta mère suce des bite en enfert, t'en est qu'à la moitié là ^ ^"

#recuperation du nouveau noyau
cd /opt/kernel
if [ $IS_HOST == "true" ]; then
	cecho yellow "Téléchargement des noyaux 9.05 pour vm 9.2"
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Linux/OS_VM/v9/linux-image-2.6.38.8-vm-9.05_0_i386.deb
	if [ $? != 0 ]; then
	    cecho -red "Failed to download linux-image-2.6.38.8-vm-9.05_0_i386.deb from the FTP ! res=$?"
	    exit 1
	fi
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Linux/OS_VM/v9/linux-headers-2.6.38.8-vm-9.05_0_i386.deb
	if [ $? != 0 ]; then
	    cecho -red "Failed to download linux-headers-2.6.38.8-vm-9.05_0_i386.deb from the FTP ! res=$?"
	    exit 1
	fi
	cecho yellow "Installation des noyaux 9.05 pour vm 9.2"
	dpkg -i linux-image-2.6.38.8-vm-9.05_0_i386.deb linux-headers-2.6.38.8-vm-9.05_0_i386.deb
else
	cecho yellow "Téléchargement des noyaux 9.05 pour target 9.2"
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Linux/OS_Target/v9/linux-image-2.6.38.8-target-9.05_0_i386.deb
	if [ $? != 0 ]; then
	    cecho -red "Failed to download linux-image-2.6.38.8-target-9.05_0_i386.deb from the FTP ! res=$?"
	    exit 1
	fi
	cecho yellow "Installation des noyaux 9.05 pour target 9.2"
	dpkg -i linux-image-2.6.38.8-target-9.05_0_i386.deb
fi


#recuperation du nouvel eclipse 
if [ $IS_HOST == "true" ]; then
	cd /usr/bin
	#download de l'archive
	cecho yellow "Téléchargement d'eclipse pour vm 9.2"
	wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Dependance/Eclipse/eclipse_indigo-ard_9.2_i386.tgz
	if [ $? != 0 ]; then
	    cecho -red "Failed to download eclipse_indigo-ard_9.2_i386.tgz from the FTP ! res=$?"
	    exit 1
	fi

	#extraction
	cecho yellow "Installation d'eclipse pour vm 9.2"
	tar -xf eclipse_indigo-ard_9.2_i386.tgz
	if [ $? != 0 ]; then
	    cecho red "Failed to untar eclipse_indigo-ard_9.2_i386.tgz ! res=$?"
	    exit 1
	fi
	rm eclipse_indigo-ard_9.2_i386.tgz -f
fi


#preparation du prochain patch
cd /opt/kernel
wget ftp://ard_user:robotik@wixibox/34%20-%20Info/Publication/livraison_v9.0/patch_9.2/check_update-9.2.0.sh
if [ $? != 0 ]; then
    cecho red "Failed to get next patch check"
    exit 1
fi
ln -sf check_update-9.2.0.sh check_update.sh
dos2unix check_update-9.2.0.sh

#succes
cecho green "SUCCEED !!!"
cecho green "please reboot and reinstall VM additions and then retry ard-update to check next update"
echo "9.2" > /opt/kernel/ard-vm-version 


