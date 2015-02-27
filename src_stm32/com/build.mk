#Specific message for ARD use case
obj-stm32-stm32_ard += com/msgs/ConfigurationMsg.o
obj-stm32-stm32_ard += com/msgs/EventMessage.o
obj-stm32-stm32_ard += com/msgs/FaultMessage.o
obj-stm32-stm32_ard += com/msgs/GyroMsg.o
obj-stm32-stm32_ard += com/msgs/HokuyoMessage.o
obj-stm32-stm32_ard += com/msgs/LogMessage.o
#obj-stm32-stm32_ard += com/msgs/OpponentListMsg.o
obj-stm32-stm32_ard += com/msgs/StatusMessage.o
obj-stm32-stm32_ard += com/msgs/VersionMessage.o
obj-stm32-stm32_ard += com/msgs/X86CmdMsg.o

#Stack de communication (decoupe le flux en messages avec entêtes)
obj-stm32-stm32_ard += com/stack_com/ArdCom.o
obj-stm32-stm32_ard += com/stack_com/Datagram.o
obj-stm32-stm32_ard += com/stack_com/IpcHeader.o
obj-stm32-stm32_ard += com/stack_com/IpcMsg.o
obj-stm32-stm32_ard += com/stack_com/heartbeat.o

#Driver USB
obj-stm32-stm32_ard += com/usb/circular_buffer.o
obj-stm32-stm32_ard += com/usb/usb.o
obj-stm32-stm32_ard += com/usb/usb_cb.o
obj-stm32-stm32_ard += com/usb/usb_mutex.o
obj-stm32-stm32_ard += com/usb/usb_descriptor.o
obj-stm32-stm32_ard += com/usb/driver_ST/usb_core.o
obj-stm32-stm32_ard += com/usb/driver_ST/usb_dcd.o
obj-stm32-stm32_ard += com/usb/driver_ST/usb_dcd_int.o
obj-stm32-stm32_ard += com/usb/driver_ST/usbd_core.o
obj-stm32-stm32_ard += com/usb/driver_ST/usbd_ioreq.o
obj-stm32-stm32_ard += com/usb/driver_ST/usbd_req.o
obj-stm32-stm32_ard += com/usb/driver_ST/usbd_usr.o



