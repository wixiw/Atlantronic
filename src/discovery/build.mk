obj-discovery-stm32_ard  += kernel/main.o
obj-discovery-stm32_ard += kernel/asm/isr_stm32f4xx.o
obj-discovery-stm32_ard += kernel/rcc.o
obj-discovery-stm32_ard += kernel/systick.o
obj-discovery-stm32_ard += kernel/tasks.o
obj-discovery-stm32_ard += kernel/queue.o
obj-discovery-stm32_ard += kernel/port.o
obj-discovery-stm32_ard += kernel/list.o
obj-discovery-stm32_ard += kernel/heap_1.o
obj-discovery-stm32_ard += kernel/log.o
obj-discovery-stm32_ard += kernel/heartbeat.o
obj-discovery-stm32_ard += kernel/driver/usb/usb.o
obj-discovery-stm32_ard += kernel/driver/usb/usb_descriptor.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usb_core.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usb_dcd.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usb_dcd_int.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_core.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_req.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_atlantronic_core.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_usr.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_desc.o
obj-discovery-stm32_ard += kernel/state_machine/state_machine.o
obj-discovery-stm32_ard += kernel/driver/power.o
obj-discovery-stm32_ard += kernel/driver/usart.o
obj-discovery-stm32_ard += kernel/driver/spi.o
obj-discovery-stm32_ard += kernel/driver/gyro.o
obj-discovery-stm32_ard += kernel/driver/hokuyo.o
obj-discovery-stm32_ard += kernel/driver/dynamixel.o
obj-discovery-stm32_ard += kernel/driver/pwm.o
obj-discovery-stm32_ard += kernel/driver/encoder.o
obj-discovery-stm32_ard += kernel/driver/adc.o
obj-discovery-stm32_ard += kernel/math/vect_plan.o
obj-discovery-stm32_ard += kernel/math/vect2.o
obj-discovery-stm32_ard += kernel/math/simpson_integrator.o
obj-discovery-stm32_ard += kernel/math/regression.o
obj-discovery-stm32_ard += kernel/pump.o
obj-discovery-stm32_ard += kernel/hokuyo_tools.o

#discovery folder
obj-discovery-stm32_ard += discovery/gpio.o
obj-discovery-stm32_ard += discovery/Signal.o
obj-discovery-stm32_ard += discovery/boot_signals.o
obj-discovery-stm32_ard += discovery/location.o
obj-discovery-stm32_ard += discovery/table_2015.o

#ipc folder
obj-discovery-stm32_ard += ipc/Datagram.o
obj-discovery-stm32_ard += ipc/IpcHeader.o
obj-discovery-stm32_ard += ipc/IpcMsg.o

#stm32_tasks folder
obj-discovery-stm32_ard += stm32_tasks/fault.o
obj-discovery-stm32_ard += stm32_tasks/end.o
obj-discovery-stm32_ard += stm32_tasks/led.o
obj-discovery-stm32_ard += stm32_tasks/control.o
obj-discovery-stm32_ard += stm32_tasks/detection.o

bin-discovery += stm32_ard
