obj-bar-us += kernel/main.o
obj-bar-us += kernel/asm/isr.o
obj-bar-us += kernel/tasks.o
obj-bar-us += kernel/list.o
obj-bar-us += kernel/queue.o
obj-bar-us += kernel/port.o
obj-bar-us += kernel/systick.o
obj-bar-us += kernel/heap_3.o
obj-bar-us += kernel/syscalls.o
obj-bar-us += kernel/rcc.o
obj-bar-us += kernel/driver/can.o
obj-bar-us += kernel/error.o
#obj-bar-us += kernel/end.o
obj-bar-us += bar/gpio.o
obj-bar-us += bar/us.o
bin-bar += us

obj-bar-us_hokuyo += kernel/main.o
obj-bar-us_hokuyo += kernel/asm/isr.o
obj-bar-us_hokuyo += kernel/tasks.o
obj-bar-us_hokuyo += kernel/list.o
obj-bar-us_hokuyo += kernel/queue.o
obj-bar-us_hokuyo += kernel/port.o
obj-bar-us_hokuyo += kernel/systick.o
obj-bar-us_hokuyo += kernel/heap_3.o
obj-bar-us_hokuyo += kernel/syscalls.o
obj-bar-us_hokuyo += kernel/rcc.o
obj-bar-us_hokuyo += kernel/driver/can.o
obj-bar-us_hokuyo += kernel/driver/usart.o
obj-bar-us_hokuyo += kernel/driver/hokuyo.o
obj-bar-us_hokuyo += kernel/hokuyo_tools.o
#obj-bar-us_hokuyo += kernel/end.o
obj-bar-us_hokuyo += kernel/error.o
obj-bar-us_hokuyo += bar/gpio.o
obj-bar-us_hokuyo += kernel/vect_pos.o
bin-bar += us_hokuyo

obj-bar-test_log += kernel/main.o
obj-bar-test_log += kernel/asm/isr.o
obj-bar-test_log += kernel/tasks.o
obj-bar-test_log += kernel/list.o
obj-bar-test_log += kernel/queue.o
obj-bar-test_log += kernel/port.o
obj-bar-test_log += kernel/systick.o
obj-bar-test_log += kernel/heap_3.o
obj-bar-test_log += kernel/syscalls.o
obj-bar-test_log += kernel/rcc.o
obj-bar-test_log += kernel/log.o
obj-bar-test_log += kernel/error.o
obj-bar-test_log += kernel/utf8.o
obj-bar-test_log += kernel/driver/usb/usb.o
obj-bar-test_log += kernel/driver/usb/usb_init.o
obj-bar-test_log += kernel/driver/usb/usb_prop.o
obj-bar-test_log += kernel/driver/usb/usb_desc.o
obj-bar-test_log += kernel/driver/usb/usb_core.o
obj-bar-test_log += kernel/driver/usb/usb_pwr.o
obj-bar-test_log += kernel/driver/usb/usb_istr.o
obj-bar-test_log += kernel/driver/usb/usb_sil.o
obj-bar-test_log += kernel/driver/usb/otgd_fs_dev.o
obj-bar-test_log += kernel/driver/usb/otgd_fs_cal.o
obj-bar-test_log += kernel/driver/usb/otgd_fs_int.o
obj-bar-test_log += kernel/driver/usb/otgd_fs_pcd.o
obj-bar-test_log += bar/gpio.o
obj-bar-test_log += kernel/test/log/task1.o
bin-bar += test_log