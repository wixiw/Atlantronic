obj-linux-usb_interface += linux/tools/usb_interface.o
obj-linux-usb_interface += linux/tools/com.o
obj-linux-usb_interface += kernel/hokuyo_tools.o
obj-linux-usb_interface += kernel/math/regression.o
obj-linux-usb_interface += kernel/vect_pos.o
obj-linux-usb_interface += kernel/math/vect2.o
obj-linux-usb_interface += kernel/math/fx_math.o
obj-linux-usb_interface += linux/tools/cli.o
obj-linux-usb_interface += linux/tools/cmd.o
obj-linux-usb_interface += linux/tools/robot_interface.o
obj-linux-usb_interface += linux/tools/qemu.o
obj-linux-usb_interface += foo/table.o
lib-linux-usb_interface += -lm -lreadline
bin-linux += usb_interface

obj-linux-glplot += linux/tools/glplot.o
obj-linux-glplot += linux/tools/com.o
obj-linux-glplot += kernel/hokuyo_tools.o
obj-linux-glplot += kernel/math/regression.o
obj-linux-glplot += kernel/math/fx_math.o
obj-linux-glplot += kernel/vect_pos.o
obj-linux-glplot += linux/tools/cli.o
obj-linux-glplot += linux/tools/cmd.o
obj-linux-glplot += linux/tools/graphique.o
obj-linux-glplot += linux/tools/joystick.o
obj-linux-glplot += foo/graph.o
obj-linux-glplot += foo/table.o
obj-linux-glplot += linux/tools/robot_interface.o
obj-linux-glplot += linux/tools/qemu.o
cflags-linux-linux/tools/glplot.o+=$(shell pkg-config --cflags gtk+-2.0 gtkglext-1.0)
lib-linux-glplot+=$(shell pkg-config --libs gtk+-2.0 gtkglext-1.0) -lreadline -lm
bin-linux += glplot

obj-linux-sin_table_gen += linux/tools/sin_table_gen.o
lib-linux-sin_table_gen += -lm
bin-linux += sin_table_gen

obj-linux-graph_gen += linux/tools/graph_gen.o
obj-linux-graph_gen += kernel/math/fx_math.o
lib-linux-graph_gen += -lm
bin-linux += graph_gen
