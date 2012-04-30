#!/bin/sh


### BEGIN INIT INFO
# Provides:          boot_read_only
# Required-Start:    
# Required-Stop:     
# Should-Start:         
# Should-stop:
# Default-Start:     S
# Default-Stop:
# X-Interactive:     true
# Short-Description: Boot in read only mode.
### END INIT INFO



do_start () {
	echo "start boot_read_only : montage unionfs"
	/bin/mount -n -t tmpfs tmpfs /unionfs/etc -o size=32M
	/bin/mount -n -t unionfs unionfs /etc -o dirs=/unionfs/etc=rw:/etc=ro 
	/bin/mount -n -t tmpfs tmpfs /unionfs/var -o size=128M
	/bin/mount -n -t unionfs unionfs /var -o dirs=/unionfs/var=rw:/var=ro 
	/bin/mount -n -t tmpfs tmpfs /unionfs/opt -o size=600M
	/bin/mount -n -t unionfs unionfs /opt -o dirs=/unionfs/opt=rw:/opt=ro 
	/bin/mount -n -t tmpfs tmpfs /unionfs/tmp -o size=32M
	/bin/mount -n -t unionfs unionfs /tmp -o dirs=/unionfs/tmp=rw:/tmp=ro 
	echo "tmpfs /unionfs/etc tmpfs rw,size=32M 0 0" >> /etc/mtab
	echo "unionfs /etc unionfs rw,dirs=/unionfs/etc=rw:/etc=ro 0 0" >> /etc/mtab
	echo "tmpfs /unionfs/var tmpfs rw,size=128M 0 0" >> /etc/mtab
	echo "unionfs /var unionfs rw,dirs=/unionfs/var=rw:/var=ro 0 0" >> /etc/mtab
	echo "tmpfs /unionfs/opt tmpfs rw,size=256M 0 0" >> /etc/mtab
	echo "unionfs /opt unionfs rw,dirs=/unionfs/opt=rw:/opt=ro 0 0" >> /etc/mtab
	echo "tmpfs /unionfs/tmp tmpfs rw,size=256M 0 0" >> /etc/mtab
	echo "unionfs /tmp unionfs rw,dirs=/unionfs/tmp=rw:/tmp=ro 0 0" >> /etc/mtab
}


case "$1" in
  start)
        do_start
        ;;
  restart|reload|force-reload)
        echo "Error: argument '$1' not supported" >&2
        exit 3
        ;;
  stop)
        echo "stop boot_read_only : Rien a faire"
        ;;
  rw) 
	echo "Transition to RW in progress..."
	mount -w -o remount /
	sleep 1
	umount -n /etc
	#umount -n /var not unmounted because programs are using it and no need to change in that directory
	umount -n /tmp
	umount -n /opt
	echo "Back to Read-Write mode"
	echo "Ne pas brancher de cle USB avant de repasser en Read-Only !!"
	echo "BUG : type 'umount -n /etc' and don't stay in /etc!"
	exit 3
	;;
  ro)
	echo "Return to Read-Only mode"
	sync
	mount -n -t unionfs unionfs /etc -o dirs=/unionfs/etc=rw:/etc=ro
	#mount -n -t unionfs unionfs /var -o dirs=/unionfs/var=rw:/var=ro
	mount -n -t unionfs unionfs /opt -o dirs=/unionfs/opt=rw:/opt=ro
	mount -n -t unionfs unionfs /tmp -o dirs=/unionfs/tmp=rw:/tmp=ro
	mount -r -o remount /
	echo "C'est bon, on peut rebrancher des cles USB"
	exit 3	
	;;
  *)
        echo "Usage: $0 start|stop|rw|ro" >&2
        exit 3
        ;;
esac

:

