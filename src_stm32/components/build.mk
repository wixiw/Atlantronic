#Dynamixel
obj-stm32-stm32_ard += components/dynamixel/dynamixel.o

#Gyro
obj-stm32-stm32_ard += components/gyro/simpson_integrator.o
obj-stm32-stm32_ard += components/gyro/spi.o
obj-stm32-stm32_ard += components/gyro/gyro.o

#LED
obj-stm32-stm32_ard += components/led/led.o

#Localisation
obj-stm32-stm32_ard += components/localization/detection.o
obj-stm32-stm32_ard += components/localization/hokuyo.o
obj-stm32-stm32_ard += components/localization/hokuyo_tools.o
obj-stm32-stm32_ard += components/localization/location.o
obj-stm32-stm32_ard += components/localization/regression.o
obj-stm32-stm32_ard += components/localization/segment_intersection.o
obj-stm32-stm32_ard += components/localization/table_2015.o
obj-stm32-stm32_ard += components/localization/vect_pos.o
obj-stm32-stm32_ard += components/localization/vect_plan.o
obj-stm32-stm32_ard += components/localization/vect2.o


#Log
obj-stm32-stm32_ard += components/log/log.o
#TODO : obj-stm32-stm32_ard += components/log/fault.o (buggé)

#Robot
obj-stm32-stm32_ard += components/robot/color.o
obj-stm32-stm32_ard += components/robot/power.o

#Pump
obj-stm32-stm32_ard += components/pump/pwm.o
obj-stm32-stm32_ard += components/pump/pump.o
