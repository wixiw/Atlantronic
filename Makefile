help:
	@echo "------------------------------------------------"
	@echo "Please choose one of the following :"
	@echo " - stm32   : build stm32_ard programme which goes on the slave board interfacing HW."
	@echo " - debug   : build stm32 program in debug mode."
	@echo " - x86     : build host utilities to debug stm32_ard program."
	@echo " - modules : build the linux usb module to connect to the stm32 board. You must follow the call with 'sudo make install'."
	@echo " - clean   : delete everything that was produced."
	@echo " - all     : build stm32 and x86."
	@echo "------------------------------------------------"

debug:
	@echo "[stm32 debug]"
	+DEBUG=1 make -C src_stm32


stm32:
	@echo "[stm32]"
	+make -C src_stm32
	
x86:
	@echo "[x86]"
	+make -C src_x86
	
module:
	@echo "[X86/Module]"
	+make -C src_x86 modules

install:
	+make -C src_x86 modules_install

clean:
	@rm bin obj -Rf
	@echo "Cleaned everything in folder obj and bin."

all: stm32 x86
