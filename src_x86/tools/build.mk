obj-x86-fifo += ../src_stm32/com/stack_com/Datagram.o
obj-x86-fifo += ../src_stm32/com/stack_com/IpcHeader.o
obj-x86-fifo += ../src_stm32/com/stack_com/IpcMsg.o
obj-x86-fifo += ../src_stm32/com/msgs/ConfigurationMsg.o
obj-x86-fifo += ../src_stm32/com/msgs/EventMessage.o
obj-x86-fifo += ../src_stm32/com/msgs/FaultMessage.o
obj-x86-fifo += ../src_stm32/com/msgs/GyroMsg.o
obj-x86-fifo += ../src_stm32/com/msgs/HokuyoMessage.o
obj-x86-fifo += ../src_stm32/com/msgs/LogMessage.o
obj-x86-fifo += ../src_stm32/com/msgs/OpponentListMsg.o
obj-x86-fifo += ../src_stm32/com/msgs/RawMessage.o
obj-x86-fifo += ../src_stm32/com/msgs/StatusMessage.o
obj-x86-fifo += ../src_stm32/com/msgs/VersionMessage.o
obj-x86-fifo += ../src_stm32/com/msgs/X86CmdMsg.o
#X86 specific
obj-x86-fifo += ../src_stm32/com/msgs/MessagePrinter.o


obj-x86-fifoWriter = $(obj-x86-fifo)
obj-x86-fifoWriter += tools/fifoWriter.o
lib-x86-fifoWriter += -lm
bin-x86 += fifoWriter


obj-x86-fifoReader = $(obj-x86-fifo)
obj-x86-fifoReader += tools/fifoReader.o
lib-x86-fifoReader += -lm
bin-x86 += fifoReader

obj-x86-bootScenario = $(obj-x86-fifo)
obj-x86-bootScenario += tools/bootScenario.o
lib-x86-bootScenario += -lm
bin-x86 += bootScenario
