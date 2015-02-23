# Core 
obj-discovery-core += kernel/main.o
obj-discovery-core += kernel/asm/isr_stm32f4xx.o
obj-discovery-core += kernel/rcc.o
obj-discovery-core += kernel/systick.o
obj-discovery-core += kernel/tasks.o
obj-discovery-core += kernel/queue.o
obj-discovery-core += kernel/port.o
obj-discovery-core += kernel/list.o
obj-discovery-core += kernel/heap_1.o
obj-discovery-core += kernel/driver/power.o
obj-discovery-core += kernel/driver/usart.o
obj-discovery-core += kernel/driver/encoder.o
obj-discovery-core += kernel/driver/adc.o
obj-discovery-core += kernel/math/regression.o
obj-discovery-core += discovery/gpio.o
obj-discovery-core += discovery/Signal.o
obj-discovery-core += discovery/boot_signals.o

#LED
obj-discovery-led += stm32_tasks/led.o

#Communication X86<=>STM32
#ipc stack
obj-discovery-usb_stack += ipc/Datagram.o
obj-discovery-usb_stack += ipc/IpcHeader.o
obj-discovery-usb_stack += ipc/IpcMsg.o
obj-discovery-usb_stack += ipc_disco/EventMessage.o

#driver USB ST-ARD wrapper
obj-discovery-usb_stack += kernel/driver/usb/circular_buffer.o
obj-discovery-usb_stack += kernel/driver/usb/usb.o
obj-discovery-usb_stack += kernel/driver/usb/usb_cb.o
obj-discovery-usb_stack += kernel/driver/usb/usb_mutex.o
obj-discovery-usb_stack += kernel/driver/usb/usb_descriptor.o
##driver USB from ST
obj-discovery-usb_stack += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd.o
obj-discovery-usb_stack += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd_ex.o
obj-discovery-usb_stack += kernel/driver/usb/stm32f4xx/stm32f4xx_ll_usb.o
obj-discovery-usb_stack += kernel/driver/usb/stm32f4xx/usbd_conf.o
obj-discovery-usb_stack += kernel/driver/usb/stm32f4xx/usbd_core.o
obj-discovery-usb_stack += kernel/driver/usb/stm32f4xx/usbd_ctlreq.o
obj-discovery-usb_stack += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
obj-discovery-usb_stack += kernel/heartbeat.o

#Log
obj-discovery-log += ipc_disco/LogMessage.o
obj-discovery-log += kernel/log.o
#Fault
obj-discovery-fault += stm32_tasks/fault.o
obj-discovery-fault += ipc_disco/FaultMessage.o

#Gyro
obj-discovery-gyro += kernel/math/simpson_integrator.o
obj-discovery-gyro += kernel/driver/spi.o
obj-discovery-gyro += kernel/driver/gyro.o
obj-discovery-gyro += ipc_disco/GyroMsg.o

#Localisation
obj-discovery-localization += kernel/math/vect_plan.o
obj-discovery-localization += kernel/math/vect2.o
obj-discovery-localization += discovery/location.o
obj-discovery-localization += kernel/driver/hokuyo.o
obj-discovery-localization += stm32_tasks/detection.o
obj-discovery-localization += ipc_disco/OpponentListMsg.o
obj-discovery-localization += ipc_disco/HokuyoMessage.o
obj-discovery-localization += kernel/hokuyo_tools.o
obj-discovery-localization += discovery/table_2015.o

#Dynamixel
obj-discovery-dynamixel += kernel/driver/dynamixel.o

#Pump
obj-discovery-pump += kernel/driver/pwm.o
obj-discovery-pump += kernel/pump.o

#Match
obj-discovery-match += stm32_tasks/end.o
obj-discovery-match += stm32_tasks/control.o
obj-discovery-match += kernel/state_machine/state_machine.o
obj-discovery-match += kernel/driver/usb/ArdCom.o
obj-discovery-match += kernel/driver/usb/ArdCom_c_wrapper.o
obj-discovery-match += ipc_disco/ConfigurationMsg.o
obj-discovery-match += ipc_disco/StatusMessage.o
obj-discovery-match += ipc_disco/VersionMessage.o
obj-discovery-match += ipc_disco/X86CmdMsg.o


obj-discovery-stm32_ard += $(obj-discovery-core)
obj-discovery-stm32_ard += $(obj-discovery-led)
obj-discovery-stm32_ard += $(obj-discovery-usb_stack)
obj-discovery-stm32_ard += $(obj-discovery-log)
#obj-discovery-stm32_ard += $(obj-discovery-fault) //bugged (and not usefull)
obj-discovery-stm32_ard += $(obj-discovery-gyro)
#obj-discovery-stm32_ard += $(obj-discovery-localization) //not usefull yet
obj-discovery-stm32_ard += $(obj-discovery-pump)
obj-discovery-stm32_ard += $(obj-discovery-match)
bin-discovery += stm32_ard
