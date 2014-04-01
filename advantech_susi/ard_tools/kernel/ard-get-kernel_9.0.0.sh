#/bin/bash

# author WLA
# date 12/11/2011
# version 
SCRIPT_VERSION="9.0.0"

# this script aim at getting the last kernel sources and add ard patches

#################################################################################
# Conf
#################################################################################

LINUX_VERSION="2.6.38.8"
XENOMAI_VERSION="2.6.0"
ARD_MAJOR="9"
ARD_OS_VERSION="$ARD_MAJOR.57"
XENOMAI_ARD_VERSION="$XENOMAI_VERSION-ard1"
FTP_INFO_DIR="ftp://ard_user:robotik@88.191.124.77/34%20-%20Info"
FTP_DEPENCE_DIR="$FTP_INFO_DIR/Dependance"
UNIONFS_PATCH="unionfs-2.5.10_for_$LINUX_VERSION.diff"
ADEOS_PATCH="adeos-ipipe-$LINUX_VERSION-x86-2.10-01.patch"
SJA1000_PATCH="wg_platform.patch"
KERNEL_TARGET_CONFIG="config_$LINUX_VERSION-target-$ARD_OS_VERSION"
KERNEL_VM_CONFIG="config_$LINUX_VERSION-vm-$ARD_OS_VERSION"
LOG_FILE="`pwd`/ard_get_kernel.log"

#################################################################################
# functions
#################################################################################

#a utiliser pour tester le resultat d'une commande et generer un message d'erreur
# test_res $? "mon message d'erreur"
function test_res
{
	ROUGE="\\033[1;31m"
	NORMAL="\\033[0;39m"

	if [ $1 != 0 ]; then
	    echo -e $ROUGE "Error : " $2  $NORMAL
    		exit 1
	fi
}

#cree les dossiers pour ranger ce qui sera download
function create_dirs
{
	mkdir /home/ard/src -p
	mkdir /home/ard/src/patches -p
	mkdir /home/ard/src/linux-$LINUX_VERSION -p
}

#recupere les patches necessaires pour builder le noyau ard
function get_patches
{
	cd /home/ard/src/patches

	if [ ! -e $UNIONFS_PATCH ]; then
		echo "Downloading unionfs $UNIONFS_PATCH ..."
		wget $FTP_DEPENCE_DIR/Unionfs/$UNIONFS_PATCH.gz  >> $LOG_FILE
		test_res $? "Failed to download unionfs patch" 

		gunzip $UNIONFS_PATCH.gz
		test_res $? "Failed to extract Unionfs patch"
	else
		echo "Unionsfs patch $UNIONFS_PATCH already present."
	fi
	
	if [ ! -e $ADEOS_PATCH ]; then
		echo "Downloading adeos $ADEOS_PATCH ..."
		wget $FTP_DEPENCE_DIR/Xenomai/xenomai-$XENOMAI_ARD_VERSION/$ADEOS_PATCH >> $LOG_FILE
		test_res $? "Failed to download adeos patch" 
	else
		echo "Adeos patch $ADEOS_PATCH already present."
	fi

	if [ ! -e $SJA1000_PATCH ]; then
		echo "Downloading adeos $SJA1000_PATCH ..."
		wget $FTP_DEPENCE_DIR/Can/SocketCan/$SJA1000 >> $LOG_FILE
		test_res $? "Failed to download sja1000 patch" 
	else
		echo "SJA1000 patch $SJA1000_PATCH already present."
	fi
}

#recupere les sources du noyau
function get_linux_sources
{
	cd /home/ard/src/linux-$LINUX_VERSION

	if [ ! -e Kbuild ]; then
		echo "Downloading linux $LINUX_VERSION sources..."		
		ketchup -G $LINUX_VERSION >> $LOG_FILE
		test_res $? "Failed to put ketchup on it !" 
	else
		echo "Linux $LINUX_VERSION already preset"
	fi
}

#applique les patchs sur le noyau
function apply_patches
{
	cd /home/ard/src/linux-$LINUX_VERSION

	patch -p1 < ../patches/$UNIONFS_PATCH >> $LOG_FILE
	test_res $? "Failed to patch kernel with unionfs patch"

	cd /home/ard/src/xenomai-2.6.0
	./scripts/prepare-kernel.sh --linux=/home/ard/src/linux-$LINUX_VERSION --adeos=/home/ard/src/patches/$ADEOS_PATCH --arch=x86
	test_res $? "Failed to patch kernel for xenomai"

 
}

#recupere les sources xenomai
function get_xenomai_sources
{
	cd /home/ard/src/

	if [ ! -e xenomai-$XENOMAI_VERSION.tar.bz2 ]; then
		echo "Downloading Xenomai sources..."
		wget http://download.gna.org/xenomai/stable/xenomai-$XENOMAI_VERSION.tar.bz2  >> $LOG_FILE
		test_res $? "Failed to download Xenomai $XENOMAI_VERSION" 
		tar -xjf xenomai-*.tar.bz2  >> $LOG_FILE
		test_res $? "Failed to extract Xenomai $XENOMAI_VERSION sources" 
	else
		echo "Xenomai sources already present"
	fi
}

#build xenomai
function build_xenomai
{
	cd /home/ard/src/xenomai-$XENOMAI_VERSION

	sed -i "s/build: build-arch build-indep/CONFIG_OPTS +=  --enable-dlopen-skins\n\nbuild: build-arch build-indep/" debian/rules  >> $LOG_FILE
	test_res $? "Failed to add --enable-dlopen-skins compile option to Xenomai" 

	DEBEMAIL="contact@team-ard.com" DEBFULLNAME="A.R.D." debchange -v $XENOMAI_ARD_VERSION Release  >> $LOG_FILE $XENOMAI_ARD_VERSION
	test_res $? "Failed to create debmail entry" 

	debuild -uc -us >> $LOG_FILE 
	test_res $? "Failed to build debian packages" 

	#rangement des paquets debian generes
	mkdir /home/ard/src/deb -p 
	mkdir /home/ard/src/deb/xenomai-$XENOMAI_ARD_VERSION -p
	cd ..
	mv *xenomai*.deb /home/ard/src/deb/xenomai-$XENOMAI_ARD_VERSION

	#effecament des fichiers non utiles
	rm *xenomai*.dsc *xenomai*.build *xenomai*.changes *xenomai*.tar.gz
}

#build le noyau linux
function build_kernel
{
	cd /home/ard/src/linux-$LINUX_VERSION
	
	#pour la cible
	if [ ! -e $KERNEL_TARGET_CONFIG ]; then
		echo "Downloading kernel config $KERNEL_TARGET_CONFIG  ..."
		wget $FTP_INFO_DIR/Dependance/Linux/OS_Target/v$ARD_MAJOR/$KERNEL_TARGET_CONFIG>> $LOG_FILE
		test_res $? "Failed to download kernel config patch" 
	else
		echo "Kernel config patch $KERNEL_TARGET_CONFIG already present"
	fi
	cp $KERNEL_TARGET_CONFIG .config
	test_res $? "Failed to copy target kernel config"
	#time CONCURRENCY_LEVEL=4 fakeroot make-kpkg kernel_image --revision=0
	#test_res $? "Failed to build target kernel" 
	#mv ../*.deb ../deb

	#pour la vm
	if [ ! -e $KERNEL_VM_CONFIG ]; then
		echo "Downloading kernel config $KERNEL_VM_CONFIG_PATCH  ..."
		wget $FTP_INFO_DIR/Dependance/Linux/OS_VM/v$ARD_MAJOR/$KERNEL_VM_CONFIG>> $LOG_FILE
		test_res $? "Failed to download kernel config patch" 
	else
		echo "Kernel config patch $KERNEL_VM_CONFIG_PATCH already present"
	fi
	cp $KERNEL_VM_CONFIG .config
	test_res $? "Failed to copy vm kernel config"
	time CONCURRENCY_LEVEL=4 fakeroot make-kpkg kernel_image kernel_headers --revision=0
	test_res $? "Failed to build vm kernel" 
	mv ../*.deb ../deb
}

function publish_xenomai
{
	ping wixibox -c 1
	test_res $? "Failed to ping wixibox, check network" 

	scp -r /home/ard/src/deb/xenomai-$XENOMAI_ARD_VERSION root@wixibox:"/opt/ftp/34\ -\ Info/Dependance/Xenomai"
	test_res $? "Failed to upload xenomai deb" 
}

function publish_kernel
{
	ping wixibox -c 1
	test_res $? "Failed to ping wixibox, check network" 

	scp /home/ard/src/deb/linux-image-*-vm-$ARD_OS_VERSION*.deb root@wixibox:"/opt/ftp/34\ -\ Info/Publication/OS_VM/v$ARD_MAJOR/"
	test_res $? "Failed to upload linux vm image deb" 
	scp /home/ard/src/deb/linux-headers-*-vm-$ARD_OS_VERSION*.deb root@wixibox:"/opt/ftp/34\ -\ Info/Publication/OS_VM/v$ARD_MAJOR/"
	test_res $? "Failed to upload linux vm headers deb"

	scp /home/ard/src/deb/linux-image-*-target-$ARD_OS_VERSION*.deb root@wixibox:"/opt/ftp/34\ -\ Info/Publication/OS_Target/v$ARD_MAJOR/"
	test_res $? "Failed to upload linux target image deb"
}

#####################################################################################
# main
#####################################################################################

echo "" >> $LOG_FILE ; echo "" >> $LOG_FILE ; echo "" >> $LOG_FILE
echo "------------------------------------------------------" >> $LOG_FILE
echo `date` " : ard-get-kernel version $SCRIPT_VERSION"


create_dirs
get_patches
get_linux_sources
get_xenomai_sources
build_xenomai
apply_patches
build_kernel
publish_xenomai
publish_kernel

echo ""
echo ""
echo "ard-get-kernel END. Normally everything went good. Check $LOG_FILE to be sure"
echo "------------------------------------------------------" >> $LOG_FILE

