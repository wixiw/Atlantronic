#!/bin/bash
set -e

##################################################
# CONFIG
##################################################

#The name of the executable to launch in qemu
PROP_STM32_PROG_NAME=stm32_ard

#The path to the Atlantronic repository
PROP_ATLANTRONIC_PATH=.

#The TCP port to connect to with a remote GDB
PROP_GDB_PORT=3333

#The name of the file family to use for emulating the usb connexion
PROP_FAKE_USB_FIFO=/tmp/carte

#The name of the file family to use for emulating a link with qemu
PROP_FAKE_QEMU_FIFO=/tmp/qemu

#Define this with any content to prevent the executable in qemu to start until gdb is connected
PROP_FREEZE_AT_STARTUP="yes"

#The path to the qemu error log !
PROP_LOG_PATH=/tmp/qemu.log

#Define the priority of the qemu thread
PRIORITY=1

##################################################
# PREPARE
##################################################

#set scheduling to 
PID=$$
#chrt -p -f $PRIORITY $PID 

PROG_OPTS+=" -M atlantronic"
PROG_OPTS+=" -nodefaults"
PROG_OPTS+=" -nographic"
PROG_OPTS+=" -chardev pipe,id=foo_usb,path=$PROP_FAKE_USB_FIFO"
PROG_OPTS+=" -chardev pipe,id=foo_model,path=$PROP_FAKE_QEMU_FIFO"
PROG_OPTS+=" -kernel $PROP_ATLANTRONIC_PATH/bin/discovery/$PROP_STM32_PROG_NAME"
PROG_OPTS+=" -gdb tcp::$PROP_GDB_PORT"
if [[ $PROP_FREEZE_AT_STARTUP ]]
then
	PROG_OPTS+=" -S"
fi
PROG_OPTS+=" -D $PROP_LOG_PATH"

QEMU_CMD="$PROP_ATLANTRONIC_PATH/qemu/arm-softmmu/qemu-system-arm $PROG_OPTS"

if [[ ! -p $PROP_FAKE_USB_FIFO".in" ]]; then
	mkfifo $PROP_FAKE_USB_FIFO".in" -m 0666
fi
if [[ ! -p $PROP_FAKE_USB_FIFO".out" ]]; then
	mkfifo $PROP_FAKE_USB_FIFO".out" -m 0666
fi

if [[ ! -p $PROP_FAKE_QEMU_FIFO".in" ]]; then
	mkfifo $PROP_FAKE_QEMU_FIFO".in" -m 0666
fi
if [[ ! -p $PROP_FAKE_QEMU_FIFO".out" ]]; then
	mkfifo $PROP_FAKE_QEMU_FIFO".out" -m 0666
fi


##################################################
# RUN
##################################################

echo "Launching QEMU with :"
echo $QEMU_CMD
$QEMU_CMD
