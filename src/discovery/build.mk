obj-discovery-core += kernel/main.o
obj-discovery-core += kernel/asm/isr_stm32f4xx.o
obj-discovery-core += kernel/rcc.o
obj-discovery-core += kernel/led.o
obj-discovery-core += kernel/systick.o
obj-discovery-core += kernel/tasks.o
obj-discovery-core += kernel/queue.o
obj-discovery-core += kernel/port.o
obj-discovery-core += kernel/list.o
obj-discovery-core += kernel/heap_1.o
obj-discovery-core += kernel/log.o
obj-discovery-core += kernel/fault.o
obj-discovery-core += kernel/heartbeat.o

obj-discovery-core += kernel/driver/usb/usb.o
obj-discovery-core += kernel/driver/usb/usb_descriptor.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd_ex.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/stm32f4xx_ll_usb.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_core.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_ctlreq.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_conf.o
obj-discovery-core += kernel/state_machine/state_machine.o
obj-discovery-core += kernel/end.o
obj-discovery-core += kernel/driver/power.o
obj-discovery-core += kernel/driver/gpio.o
obj-discovery-core += kernel/driver/exti.o
obj-discovery-core += discovery/bsp.o

obj-discovery-gyro += $(obj-discovery-core)
obj-discovery-gyro += kernel/driver/spi.o
obj-discovery-gyro += kernel/driver/accelero.o
obj-discovery-gyro += kernel/driver/gyro.o
obj-discovery-gyro += kernel/math/simpson_integrator.o
obj-discovery-gyro += kernel/driver/adc.o
obj-discovery-gyro += kernel/driver/encoder.o
obj-discovery-gyro += kernel/control/kinematics.o
obj-discovery-gyro += kernel/geometric_model/geometric_model.o
obj-discovery-gyro += kernel/location/location.o
obj-discovery-gyro += kernel/math/vect_plan.o
obj-discovery-gyro += kernel/math/matrix_homogeneous.o
obj-discovery-gyro += discovery/control.o

obj-discovery-dynamixel += $(obj-discovery-core)
obj-discovery-dynamixel += kernel/driver/usart.o
obj-discovery-dynamixel += kernel/driver/dynamixel.o

obj-discovery-hokuyo += $(obj-discovery-core)
obj-discovery-hokuyo += kernel/driver/usart.o
obj-discovery-hokuyo += kernel/driver/hokuyo.o
obj-discovery-hokuyo += kernel/location/location.o
obj-discovery-hokuyo += kernel/geometric_model/geometric_model.o
obj-discovery-hokuyo += kernel/control/kinematics.o
obj-discovery-hokuyo += kernel/math/vect_plan.o

obj-discovery-baz_small += $(obj-discovery-core)
obj-discovery-baz_small += kernel/driver/usart.o
obj-discovery-baz_small += kernel/driver/spi.o
obj-discovery-baz_small += kernel/driver/accelero.o
obj-discovery-baz_small += kernel/driver/gyro.o
obj-discovery-baz_small += kernel/driver/dynamixel.o
obj-discovery-baz_small += kernel/driver/pwm.o
obj-discovery-baz_small += kernel/driver/encoder.o
obj-discovery-baz_small += kernel/driver/adc.o
obj-discovery-baz_small += kernel/math/vect_plan.o
obj-discovery-baz_small += kernel/math/vect2.o
obj-discovery-baz_small += kernel/math/simpson_integrator.o
obj-discovery-baz_small += kernel/control/kinematics.o
obj-discovery-baz_small += kernel/geometric_model/geometric_model.o
obj-discovery-baz_small += kernel/location/location.o
obj-discovery-baz_small += kernel/pump.o
obj-discovery-baz_small += kernel/arm.o
obj-discovery-baz_small += kernel/math/matrix_homogeneous.o
obj-discovery-baz_small += discovery/control.o

obj-discovery-baz += $(obj-discovery-core)
obj-discovery-baz += kernel/driver/usart.o
obj-discovery-baz += kernel/driver/spi.o
obj-discovery-baz += kernel/driver/accelero.o
obj-discovery-baz += kernel/driver/gyro.o
obj-discovery-baz += kernel/driver/can.o
obj-discovery-baz += kernel/driver/hokuyo.o
obj-discovery-baz += kernel/driver/dynamixel.o
obj-discovery-baz += kernel/driver/pwm.o
obj-discovery-baz += kernel/driver/encoder.o
obj-discovery-baz += kernel/driver/adc.o
obj-discovery-baz += kernel/driver/xbee.o
obj-discovery-baz += kernel/canopen.o
obj-discovery-baz += kernel/can_motor.o
obj-discovery-baz += kernel/math/vect_plan.o
obj-discovery-baz += kernel/math/vect2.o
obj-discovery-baz += kernel/math/simpson_integrator.o
obj-discovery-baz += kernel/control/kinematics.o
obj-discovery-baz += kernel/geometric_model/geometric_model.o
obj-discovery-baz += kernel/location/location.o
obj-discovery-baz += kernel/hokuyo_tools.o
obj-discovery-baz += kernel/pump.o
obj-discovery-baz += kernel/arm.o
obj-discovery-baz += kernel/math/matrix_homogeneous.o
obj-discovery-baz += discovery/control.o
obj-discovery-baz += discovery/motion.o


obj-discovery-baz += kernel/math/regression.o
#obj-discovery-baz += discovery/graph.o
#obj-discovery-baz += discovery/trajectory.o
obj-discovery-baz += discovery/detection.o
obj-discovery-baz += discovery/table.o
#obj-discovery-baz += discovery/test.o

bin-discovery += core
bin-discovery += gyro
bin-discovery += dynamixel
bin-discovery += hokuyo
bin-discovery += baz_small
bin-discovery += baz
