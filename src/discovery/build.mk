#driver USB ST-ARD wrapper
obj-discovery-stm32_ard += kernel/driver/usb/usb.o
obj-discovery-stm32_ard += kernel/driver/usb/usb_descriptor.o
#driver USB from ST
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd_ex.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/stm32f4xx_ll_usb.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_conf.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_core.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_ctlreq.o
obj-discovery-stm32_ard += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
#driver USB - custom ARD
obj-discovery-stm32_ard += kernel/driver/usb/ArdCom.o
obj-discovery-stm32_ard += kernel/driver/usb/ArdCom_c_wrapper.o
obj-discovery-stm32_ard += kernel/driver/usb/circular_buffer.o

#autres drivers
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

#ipc_disco folder
obj-discovery-stm32_ard += ipc_disco/ConfigurationMsg.o
obj-discovery-stm32_ard += ipc_disco/EventMessage.o
obj-discovery-stm32_ard += ipc_disco/FaultMessage.o
obj-discovery-stm32_ard += ipc_disco/GyroMsg.o
obj-discovery-stm32_ard += ipc_disco/HokuyoMessage.o
obj-discovery-stm32_ard += ipc_disco/LogMessage.o
#obj-discovery-stm32_ard += ipc_disco/OpponentListMsg.o TODO ne compile pas avec _dso_handle
obj-discovery-stm32_ard += ipc_disco/RawMessage.o
obj-discovery-stm32_ard += ipc_disco/StatusMessage.o
obj-discovery-stm32_ard += ipc_disco/VersionMessage.o
obj-discovery-stm32_ard += ipc_disco/X86CmdMsg.o


#stm32_tasks folder
obj-discovery-stm32_ard += stm32_tasks/fault.o
obj-discovery-stm32_ard += stm32_tasks/led.o
obj-discovery-stm32_ard += stm32_tasks/end.o
obj-discovery-stm32_ard += stm32_tasks/control.o
#TODO a remettre quand les messages usb seront code
#obj-discovery-stm32_ard += stm32_tasks/detection.o

bin-discovery += stm32_ard
