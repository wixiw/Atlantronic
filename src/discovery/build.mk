obj-discovery-test += kernel/main.o
obj-discovery-test += kernel/asm/isr_stm32f4xx.o
obj-discovery-test += kernel/rcc.o
obj-discovery-test += kernel/systick.o
obj-discovery-test += kernel/tasks.o
obj-discovery-test += kernel/queue.o
obj-discovery-test += kernel/port.o
obj-discovery-test += kernel/list.o
obj-discovery-test += kernel/heap_1.o
obj-discovery-test += discovery/gpio.o
obj-discovery-test += kernel/log.o

obj-discovery-test += kernel/driver/usb/usb.o
obj-discovery-test += kernel/driver/usb/usb_descriptor.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usb_core.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usb_dcd.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usb_dcd_int.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_core.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_req.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_atlantronic_core.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_usr.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_desc.o

bin-discovery += test
