obj-foo-test_led += kernel/main.o
obj-foo-test_led += kernel/asm/isr.o
obj-foo-test_led += kernel/task.o
obj-foo-test_led += kernel/queue.o
obj-foo-test_led += kernel/list.o
obj-foo-test_led += kernel/port.o
obj-foo-test_led += kernel/systick.o
obj-foo-test_led += kernel/heap_3.o
obj-foo-test_led += kernel/syscalls.o
obj-foo-test_led += kernel/rcc.o
obj-foo-test_led += kernel/test/led/task1.o
obj-foo-test_led += kernel/utf8.o
obj-foo-test_led += kernel/driver/usb/usb.o
obj-foo-test_led += kernel/driver/usb/usb_init.o
obj-foo-test_led += kernel/driver/usb/usb_prop.o
obj-foo-test_led += kernel/driver/usb/usb_desc.o
obj-foo-test_led += kernel/driver/usb/usb_core.o
obj-foo-test_led += kernel/driver/usb/usb_pwr.o
obj-foo-test_led += kernel/driver/usb/usb_istr.o
obj-foo-test_led += kernel/driver/usb/usb_sil.o
obj-foo-test_led += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_led += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_led += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_led += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-test_led += foo/gpio.o
bin-foo += test_led

obj-foo-test_task += kernel/main.o
obj-foo-test_task += kernel/asm/isr.o
obj-foo-test_task += kernel/task.o
obj-foo-test_task += kernel/queue.o
obj-foo-test_task += kernel/list.o
obj-foo-test_task += kernel/port.o
obj-foo-test_task += kernel/systick.o
obj-foo-test_task += kernel/heap_3.o
obj-foo-test_task += kernel/syscalls.o
obj-foo-test_task += kernel/rcc.o
obj-foo-test_task += kernel/test/tasks/task1.o
obj-foo-test_task += kernel/test/tasks/task2.o
obj-foo-test_task += kernel/utf8.o
obj-foo-test_task += kernel/driver/usb/usb.o
obj-foo-test_task += kernel/driver/usb/usb_init.o
obj-foo-test_task += kernel/driver/usb/usb_prop.o
obj-foo-test_task += kernel/driver/usb/usb_desc.o
obj-foo-test_task += kernel/driver/usb/usb_core.o
obj-foo-test_task += kernel/driver/usb/usb_pwr.o
obj-foo-test_task += kernel/driver/usb/usb_istr.o
obj-foo-test_task += kernel/driver/usb/usb_sil.o
obj-foo-test_task += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_task += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_task += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_task += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-test_task += foo/gpio.o
bin-foo += test_task

obj-foo-test_end += kernel/main.o
obj-foo-test_end += kernel/asm/isr.o
obj-foo-test_end += kernel/task.o
obj-foo-test_end += kernel/queue.o
obj-foo-test_end += kernel/list.o
obj-foo-test_end += kernel/port.o
obj-foo-test_end += kernel/systick.o
obj-foo-test_end += kernel/heap_3.o
obj-foo-test_end += kernel/syscalls.o
obj-foo-test_end += kernel/rcc.o
obj-foo-test_end += kernel/end.o
obj-foo-test_end += kernel/test/end/task1.o
obj-foo-test_end += kernel/utf8.o
obj-foo-test_end += kernel/driver/usb/usb.o
obj-foo-test_end += kernel/driver/usb/usb_init.o
obj-foo-test_end += kernel/driver/usb/usb_prop.o
obj-foo-test_end += kernel/driver/usb/usb_desc.o
obj-foo-test_end += kernel/driver/usb/usb_core.o
obj-foo-test_end += kernel/driver/usb/usb_pwr.o
obj-foo-test_end += kernel/driver/usb/usb_istr.o
obj-foo-test_end += kernel/driver/usb/usb_sil.o
obj-foo-test_end += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_end += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_end += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_end += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-test_end += foo/gpio.o
bin-foo += test_end

obj-foo-test_deplacement += kernel/main.o
obj-foo-test_deplacement += kernel/asm/isr.o
obj-foo-test_deplacement += kernel/task.o
obj-foo-test_deplacement += kernel/list.o
obj-foo-test_deplacement += kernel/queue.o
obj-foo-test_deplacement += kernel/port.o
obj-foo-test_deplacement += kernel/systick.o
obj-foo-test_deplacement += kernel/heap_3.o
obj-foo-test_deplacement += kernel/syscalls.o
obj-foo-test_deplacement += kernel/rcc.o
obj-foo-test_deplacement += kernel/end.o
obj-foo-test_deplacement += kernel/fault.o
obj-foo-test_deplacement += kernel/driver/can.o
obj-foo-test_deplacement += kernel/canopen.o
obj-foo-test_deplacement += kernel/can_motor.o
obj-foo-test_deplacement += kernel/math/regression.o
obj-foo-test_deplacement += kernel/math/segment_intersection.o
obj-foo-test_deplacement += kernel/math/trigo.o
obj-foo-test_deplacement += kernel/math/vect2.o
obj-foo-test_deplacement += kernel/math/distance.o
obj-foo-test_deplacement += kernel/trapeze.o
obj-foo-test_deplacement += kernel/vect_pos.o
obj-foo-test_deplacement += kernel/driver/usart.o
obj-foo-test_deplacement += kernel/utf8.o
obj-foo-test_deplacement += kernel/driver/usb/usb.o
obj-foo-test_deplacement += kernel/driver/usb/usb_init.o
obj-foo-test_deplacement += kernel/driver/usb/usb_prop.o
obj-foo-test_deplacement += kernel/driver/usb/usb_desc.o
obj-foo-test_deplacement += kernel/driver/usb/usb_core.o
obj-foo-test_deplacement += kernel/driver/usb/usb_pwr.o
obj-foo-test_deplacement += kernel/driver/usb/usb_istr.o
obj-foo-test_deplacement += kernel/driver/usb/usb_sil.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-test_deplacement += foo/gpio.o
obj-foo-test_deplacement += foo/control/control.o
obj-foo-test_deplacement += foo/control/trajectory.o
obj-foo-test_deplacement += foo/graph.o
obj-foo-test_deplacement += foo/control/pid.o
obj-foo-test_deplacement += foo/detection.o
obj-foo-test_deplacement += kernel/driver/hokuyo.o
obj-foo-test_deplacement += kernel/hokuyo_tools.o
obj-foo-test_deplacement += foo/pwm.o
obj-foo-test_deplacement += foo/location/odometry.o
obj-foo-test_deplacement += foo/location/location.o
obj-foo-test_deplacement += foo/location/beacon.o
obj-foo-test_deplacement += foo/encoders.o
obj-foo-test_deplacement += foo/test/test_deplacement.o
obj-foo-test_deplacement += foo/adc.o
obj-foo-test_deplacement += foo/pince.o
obj-foo-test_deplacement += foo/ax12.o
obj-foo-test_deplacement += foo/table.o
bin-foo += test_deplacement

obj-foo-test_pince += kernel/main.o
obj-foo-test_pince += kernel/asm/isr.o
obj-foo-test_pince += kernel/task.o
obj-foo-test_pince += kernel/list.o
obj-foo-test_pince += kernel/queue.o
obj-foo-test_pince += kernel/port.o
obj-foo-test_pince += kernel/systick.o
obj-foo-test_pince += kernel/heap_3.o
obj-foo-test_pince += kernel/syscalls.o
obj-foo-test_pince += kernel/rcc.o
obj-foo-test_pince += kernel/driver/usart.o
obj-foo-test_pince += kernel/end.o
obj-foo-test_pince += kernel/driver/can.o
obj-foo-test_pince += kernel/fault.o
obj-foo-test_pince += foo/gpio.o
obj-foo-test_pince += foo/control/pid.o
obj-foo-test_pince += kernel/trapeze.o
obj-foo-test_pince += foo/pwm.o
obj-foo-test_pince += foo/encoders.o
obj-foo-test_pince += foo/ax12.o
obj-foo-test_pince += foo/pince.o
obj-foo-test_pince += foo/test/test_pince.o
obj-foo-test_pince += foo/adc.o
obj-foo-test_pince += kernel/utf8.o
obj-foo-test_pince += kernel/driver/usb/usb.o
obj-foo-test_pince += kernel/driver/usb/usb_init.o
obj-foo-test_pince += kernel/driver/usb/usb_prop.o
obj-foo-test_pince += kernel/driver/usb/usb_desc.o
obj-foo-test_pince += kernel/driver/usb/usb_core.o
obj-foo-test_pince += kernel/driver/usb/usb_pwr.o
obj-foo-test_pince += kernel/driver/usb/usb_istr.o
obj-foo-test_pince += kernel/driver/usb/usb_sil.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_pcd.o
bin-foo += test_pince

obj-foo-test_hokuyo += kernel/main.o
obj-foo-test_hokuyo += kernel/asm/isr.o
obj-foo-test_hokuyo += kernel/task.o
obj-foo-test_hokuyo += kernel/list.o
obj-foo-test_hokuyo += kernel/queue.o
obj-foo-test_hokuyo += kernel/port.o
obj-foo-test_hokuyo += kernel/systick.o
obj-foo-test_hokuyo += kernel/heap_3.o
obj-foo-test_hokuyo += kernel/syscalls.o
obj-foo-test_hokuyo += kernel/rcc.o
obj-foo-test_hokuyo += kernel/end.o
obj-foo-test_hokuyo += kernel/fault.o
obj-foo-test_hokuyo += kernel/hokuyo_tools.o
obj-foo-test_hokuyo += kernel/vect_pos.o
obj-foo-test_hokuyo += kernel/math/regression.o
obj-foo-test_hokuyo += kernel/math/segment_intersection.o
obj-foo-test_hokuyo += kernel/math/trigo.o
obj-foo-test_hokuyo += kernel/math/vect2.o
obj-foo-test_hokuyo += kernel/math/distance.o
obj-foo-test_hokuyo += kernel/utf8.o
obj-foo-test_hokuyo += kernel/driver/usb/usb.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_init.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_prop.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_desc.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_core.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_pwr.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_istr.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_sil.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-test_hokuyo += kernel/driver/usart.o
obj-foo-test_hokuyo += kernel/driver/can.o
obj-foo-test_hokuyo += kernel/driver/hokuyo.o
obj-foo-test_hokuyo += foo/detection.o
obj-foo-test_hokuyo += foo/gpio.o
obj-foo-test_hokuyo += foo/location/odometry.o
obj-foo-test_hokuyo += foo/location/location.o
obj-foo-test_hokuyo += foo/location/beacon.o
obj-foo-test_hokuyo += foo/encoders.o
obj-foo-test_hokuyo += foo/table.o
bin-foo += test_hokuyo

obj-foo-test_graph += kernel/main.o
obj-foo-test_graph += kernel/asm/isr.o
obj-foo-test_graph += kernel/task.o
obj-foo-test_graph += kernel/list.o
obj-foo-test_graph += kernel/queue.o
obj-foo-test_graph += kernel/port.o
obj-foo-test_graph += kernel/systick.o
obj-foo-test_graph += kernel/heap_3.o
obj-foo-test_graph += kernel/syscalls.o
obj-foo-test_graph += kernel/rcc.o
obj-foo-test_graph += kernel/driver/can.o
obj-foo-test_graph += kernel/canopen.o
obj-foo-test_graph += kernel/can_motor.o
obj-foo-test_graph += kernel/driver/usart.o
obj-foo-test_graph += kernel/end.o
obj-foo-test_graph += kernel/fault.o
obj-foo-test_graph += kernel/log.o
obj-foo-test_graph += kernel/utf8.o
obj-foo-test_graph += kernel/driver/usb/usb.o
obj-foo-test_graph += kernel/driver/usb/usb_init.o
obj-foo-test_graph += kernel/driver/usb/usb_prop.o
obj-foo-test_graph += kernel/driver/usb/usb_desc.o
obj-foo-test_graph += kernel/driver/usb/usb_core.o
obj-foo-test_graph += kernel/driver/usb/usb_pwr.o
obj-foo-test_graph += kernel/driver/usb/usb_istr.o
obj-foo-test_graph += kernel/driver/usb/usb_sil.o
obj-foo-test_graph += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_graph += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_graph += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_graph += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-test_graph += kernel/driver/hokuyo.o
obj-foo-test_graph += kernel/trapeze.o
obj-foo-test_graph += kernel/hokuyo_tools.o
obj-foo-test_graph += kernel/math/trigo.o
obj-foo-test_graph += kernel/math/regression.o
obj-foo-test_graph += kernel/math/segment_intersection.o
obj-foo-test_graph += kernel/math/distance.o
obj-foo-test_graph += kernel/math/vect2.o
obj-foo-test_graph += kernel/vect_pos.o
obj-foo-test_graph += foo/gpio.o
obj-foo-test_graph += foo/control/control.o
obj-foo-test_graph += foo/control/trajectory.o
obj-foo-test_graph += foo/graph.o
obj-foo-test_graph += foo/control/pid.o
obj-foo-test_graph += foo/graph.o
obj-foo-test_graph += foo/pwm.o
obj-foo-test_graph += foo/location/odometry.o
obj-foo-test_graph += foo/location/location.o
obj-foo-test_graph += foo/location/beacon.o
obj-foo-test_graph += foo/encoders.o
obj-foo-test_graph += foo/ax12.o
obj-foo-test_graph += foo/pince.o
obj-foo-test_graph += foo/arm.o
obj-foo-test_graph += foo/adc.o
obj-foo-test_graph += foo/test/test_graph.o
obj-foo-test_graph += foo/recalage.o
obj-foo-test_graph += foo/table.o
obj-foo-test_graph += foo/detection.o
bin-foo += test_graph
