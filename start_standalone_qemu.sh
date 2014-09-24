#!/bin/bash

mkfifo /tmp/qemu-test.in
mkfifo /tmp/qemu-test.out
mkfifo /tmp/carte-test.in
mkfifo /tmp/carte-test.out

VERSION_GIT=96739a5bcc6e5b608c18fa1ac371e34dbe7e2478
STM32_BIN_PATH=/opt/ard/arp_stm32/src/Atlantronic/bin/discovery/baz_small

echo "Launching qemu..."
./qemu/arm-softmmu/qemu-system-arm -M atlantronic -nodefaults -nographic -chardev pipe,id=foo_usb,path=/tmp/carte-test -chardev pipe,id=foo_model,path=/tmp/qemu-test -kernel $STM32_BIN_PATH
echo "end."
