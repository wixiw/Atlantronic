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
