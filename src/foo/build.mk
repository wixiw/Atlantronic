obj-foo-strat0 += kernel/main.o
obj-foo-strat0 += kernel/asm/isr.o
obj-foo-strat0 += kernel/tasks.o
obj-foo-strat0 += kernel/list.o
obj-foo-strat0 += kernel/queue.o
obj-foo-strat0 += kernel/port.o
obj-foo-strat0 += kernel/systick.o
obj-foo-strat0 += kernel/heap_3.o
obj-foo-strat0 += kernel/syscalls.o
obj-foo-strat0 += kernel/rcc.o
obj-foo-strat0 += kernel/driver/can.o
obj-foo-strat0 += kernel/driver/usart.o
obj-foo-strat0 += kernel/end.o
obj-foo-strat0 += kernel/fault.o
obj-foo-strat0 += kernel/log.o
obj-foo-strat0 += kernel/utf8.o
obj-foo-strat0 += kernel/driver/usb/usb.o
obj-foo-strat0 += kernel/driver/usb/usb_init.o
obj-foo-strat0 += kernel/driver/usb/usb_prop.o
obj-foo-strat0 += kernel/driver/usb/usb_desc.o
obj-foo-strat0 += kernel/driver/usb/usb_core.o
obj-foo-strat0 += kernel/driver/usb/usb_pwr.o
obj-foo-strat0 += kernel/driver/usb/usb_istr.o
obj-foo-strat0 += kernel/driver/usb/usb_sil.o
obj-foo-strat0 += kernel/driver/usb/otgd_fs_dev.o
obj-foo-strat0 += kernel/driver/usb/otgd_fs_cal.o
obj-foo-strat0 += kernel/driver/usb/otgd_fs_int.o
obj-foo-strat0 += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-strat0 += kernel/driver/hokuyo.o
obj-foo-strat0 += kernel/trapeze.o
obj-foo-strat0 += kernel/hokuyo_tools.o
obj-foo-strat0 += kernel/math/trigo.o
obj-foo-strat0 += kernel/math/regression.o
obj-foo-strat0 += kernel/math/segment_intersection.o
obj-foo-strat0 += kernel/math/vect2.o
obj-foo-strat0 += kernel/math/distance.o
obj-foo-strat0 += kernel/vect_pos.o
obj-foo-strat0 += foo/gpio.o
obj-foo-strat0 += foo/control/control.o
obj-foo-strat0 += foo/control/trajectory.o
obj-foo-strat0 += foo/graph.o
obj-foo-strat0 += foo/control/pid.o
obj-foo-strat0 += foo/graph.o
obj-foo-strat0 += foo/pwm.o
obj-foo-strat0 += foo/location/odometry.o
obj-foo-strat0 += foo/location/location.o
obj-foo-strat0 += foo/location/beacon.o
obj-foo-strat0 += foo/encoders.o
obj-foo-strat0 += foo/ax12.o
obj-foo-strat0 += foo/pince.o
obj-foo-strat0 += foo/servo.o
obj-foo-strat0 += foo/arm.o
obj-foo-strat0 += foo/adc.o
obj-foo-strat0 += foo/strat0.o
obj-foo-strat0 += foo/recalage.o
obj-foo-strat0 += foo/us.o
obj-foo-strat0 += foo/table.o
obj-foo-strat0 += foo/detection.o
bin-foo += strat0

obj-foo-calib_hokuyo += kernel/main.o
obj-foo-calib_hokuyo += kernel/asm/isr.o
obj-foo-calib_hokuyo += kernel/tasks.o
obj-foo-calib_hokuyo += kernel/list.o
obj-foo-calib_hokuyo += kernel/queue.o
obj-foo-calib_hokuyo += kernel/port.o
obj-foo-calib_hokuyo += kernel/systick.o
obj-foo-calib_hokuyo += kernel/heap_3.o
obj-foo-calib_hokuyo += kernel/syscalls.o
obj-foo-calib_hokuyo += kernel/rcc.o
obj-foo-calib_hokuyo += kernel/driver/can.o
obj-foo-calib_hokuyo += kernel/driver/usart.o
obj-foo-calib_hokuyo += kernel/end.o
obj-foo-calib_hokuyo += kernel/fault.o
obj-foo-calib_hokuyo += kernel/log.o
obj-foo-calib_hokuyo += kernel/utf8.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb_init.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb_prop.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb_desc.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb_core.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb_pwr.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb_istr.o
obj-foo-calib_hokuyo += kernel/driver/usb/usb_sil.o
obj-foo-calib_hokuyo += kernel/driver/usb/otgd_fs_dev.o
obj-foo-calib_hokuyo += kernel/driver/usb/otgd_fs_cal.o
obj-foo-calib_hokuyo += kernel/driver/usb/otgd_fs_int.o
obj-foo-calib_hokuyo += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-calib_hokuyo += foo/gpio.o
obj-foo-calib_hokuyo += kernel/trapeze.o
obj-foo-calib_hokuyo += foo/control/control.o
obj-foo-calib_hokuyo += foo/control/trajectory.o
obj-foo-calib_hokuyo += foo/graph.o
obj-foo-calib_hokuyo += foo/control/pid.o
obj-foo-calib_hokuyo += foo/pwm.o
obj-foo-calib_hokuyo += foo/location/odometry.o
obj-foo-calib_hokuyo += foo/location/location.o
obj-foo-calib_hokuyo += foo/location/beacon.o
obj-foo-calib_hokuyo += foo/encoders.o
obj-foo-calib_hokuyo += foo/ax12.o
obj-foo-calib_hokuyo += foo/pince.o
obj-foo-calib_hokuyo += foo/adc.o
obj-foo-calib_hokuyo += foo/calib/calib_hokuyo.o
obj-foo-calib_hokuyo += foo/recalage.o
obj-foo-calib_hokuyo += foo/table.o
obj-foo-calib_hokuyo += foo/us.o
obj-foo-calib_hokuyo += kernel/driver/hokuyo.o
obj-foo-calib_hokuyo += kernel/hokuyo_tools.o
obj-foo-calib_hokuyo += kernel/math/regression.o
obj-foo-calib_hokuyo += kernel/math/segment_intersection.o
obj-foo-calib_hokuyo += kernel/math/trigo.o
obj-foo-calib_hokuyo += kernel/math/vect2.o
obj-foo-calib_hokuyo += kernel/math/distance.o
obj-foo-calib_hokuyo += foo/detection.o
obj-foo-calib_hokuyo += kernel/vect_pos.o
bin-foo += calib_hokuyo
