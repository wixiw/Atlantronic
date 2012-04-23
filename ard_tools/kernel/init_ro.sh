#!/bin/bash

echo "****** ARD **********"
echo "****** ARD **********"

mount -t ext4 8:2 /opt

if [ -f /opt/conf/RO ]; then 
	echo "****** ARD ********** Mounting RO"
	echo "****** ARD **********"
	echo "****** ARD **********"
	ERROR_FILE=last_boot_error.txt

	mount -t sysfs -o nodev,noexec,nosuid none /sys
	mount -t proc -o nodev,noexec,nosuid none /proc

	/bin/mount -n -t tmpfs tmpfs /mnt -o size=64M

	# /dev/root pas encore valide
	rdev="$(mountpoint -d /)"
	ROOT_PARTITION=/mnt/rootdev
	mknod -m 600 $ROOT_PARTITION b ${rdev%:*} ${rdev#*:}

	FS_STATE=$(dumpe2fs -h ${ROOT_PARTITION} 2>/dev/null | grep 'Filesystem state:' | cut -d':' -f2 | tr -d ' ')

	if ! [ "$FS_STATE" = "clean" ]
	then
		echo "Errors on ${ROOT_PARTITION}"
		fsck -f -y ${ROOT_PARTITION}
		FSCKCODE=$?

		# create ${ERROR_FILE} on read-only partition
		mount -w -o remount /
		mkdir -p /var/log
		echo -e "4\tThe current active read only partition (${ROOT_PARTITION}) was not clean and fsck -f -y /dev/root returned ${FSCKCODE}" > /var/log/${ERROR_FILE}
		sync
		mount -r -o remount /

		if [ ${FSCKCODE} -ge 2 -a ${FSCKCODE} -le 3 ]
		then
			echo "Will now reboot."
			reboot -f
		fi
	fi

	rm ${ROOT_PARTITION}

	mkdir /mnt/old_root
	mkdir /mnt/new_root

	# il y a des problemes si on fait une union directement avec / donc on va le faire avec /mnt/old_root
	mount -n -o bind / /mnt/old_root
	/bin/mount -n -t unionfs unionfs /mnt/new_root -o dirs=/mnt=rw:/mnt/old_root=ro

	# on donne un acces depuis le nouveau root sur old_root (pour modification si necessaire) et sur tmpfs_root pour le debug
	# création des dossiers uniquement, les points de montage vont etre deplaces
	mkdir /mnt/new_root/mnt/old_root
	mkdir /mnt/new_root/mnt/lost_on_reboot

	# on se place dans new_root et on prepare le dossier mnt/.old qui va recevoir l'ancien point de montage /.
	cd /mnt/new_root
	mkdir mnt/.old

	# deplacement de sys, proc et dev (peuplé par le noyau)
	mount --move /sys /mnt/new_root/sys
	mount --move /dev /mnt/new_root/dev
	mount --move /proc /mnt/new_root/proc

	# ATTENTION : ne rien faire ici, on a deplace /proc et certains programmes ont besoin d'ouvrir
	# des fichiers sur /proc. Le traitement doit etre realise avant.
	# Ici, tout est pret pour le pivot_root

	# pivot_root sur /mnt/new_root
	# et deplacement des partitions utiles pour demonter l'ancienne racine
	#
	# L implementation de pivot_root ne garanti pas le changement de la racine (et du dossier courant) du
	# process courrant (mais semble le faire sur notre version).
	# Par consequent on realise un chroot juste apres. Il ne faut pas mettre de commandes entre le
	# pivot_root et le chroot (on est potentiellement toujours dans l'ancien "/" donc les chemins absolus sont
	# mauvais et /proc n'est plus disponible sur l'ancien "/").
	#
	# On ne lance pas init directement depuis chroot mais sh pour terminer le travail (demontage de .old). On donne a sh
	# en entree et en sortie standard dev/console et non /dev/console. Il faut un chemin relatif pour utiliser la console
	# du nouveau "/" quelque soit l'implementation de pivot_root.
	#
	# Il est indispensable de faire exec chroot puis exec /sbin/init pour lancer sh puis /sbin/init avec le pid 1. 
	pivot_root . mnt/.old
	exec chroot . sh -c \
	'mount --move /mnt/.old/mnt/old_root /mnt/old_root; \
	 mount --move /mnt/.old/mnt /mnt/lost_on_reboot; \
	 umount /mnt/.old; \
	 rmdir /mnt/.old /mnt/lost_on_reboot/old_root /mnt/lost_on_reboot/new_root; \
	 exec /sbin/init' < dev/console > dev/console 2>&1

else
	echo "****** ARD ********** Mounting RW"
	echo "****** ARD **********"
	echo "****** ARD **********"
	exec /sbin/init;
fi