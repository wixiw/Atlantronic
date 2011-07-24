#Makefile for Linux SUSI 3.02

LIB_NAME = libSUSI-3.02
LIB_LINK_NAME = $(LIB_NAME).so
LINKLIB = -Wl,-rpath,./:../lib lib/$(LIB_LINK_NAME)
CFLAGS = -Wall -Werror -O2
DEMO_PROG = demo_brightness demo_cpu demo_gpio demo_hwm demo_iic demo_smbus demo_vga demo_watchdog

% : src/%.c
	$(CC) $(CFLAGS) -o bin/$@ $< $(LINKLIB) 


#default: $(DEMO_PROG)
#build desactivated in waiting of new advantech version
default:
	

install:
	rm  /usr/lib/$(LIB_NAME).*
	cp -a $(LIB_NAME).* /usr/lib/

uninstall:
	rm  /usr/lib/$(LIB_NAME).*

clean:
	rm -f ./bin/*


