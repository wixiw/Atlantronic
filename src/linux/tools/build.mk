#ipc folder
obj-linux-fifo += ipc/Datagram.o
obj-linux-fifo += ipc/IpcHeader.o
obj-linux-fifo += ipc/IpcMsg.o

#ipc_disco folder
obj-linux-fifo += ipc_disco/ConfigurationMsg.o
obj-linux-fifo += ipc_disco/EventMessage.o
obj-linux-fifo += ipc_disco/FaultMessage.o
obj-linux-fifo += ipc_disco/GyroMsg.o
obj-linux-fifo += ipc_disco/HokuyoMessage.o
obj-linux-fifo += ipc_disco/LogMessage.o
obj-linux-fifo += ipc_disco/OpponentListMsg.o
obj-linux-fifo += ipc_disco/RawMessage.o
obj-linux-fifo += ipc_disco/StatusMessage.o
obj-linux-fifo += ipc_disco/VersionMessage.o
obj-linux-fifo += ipc_disco/X86CmdMsg.o
#X86 specific
obj-linux-fifo += ipc_disco/MessagePrinter.o


obj-linux-fifoWriter = $(obj-linux-fifo)
obj-linux-fifoWriter += linux/tools/fifoWriter.o
lib-linux-fifoWriter += -lm
bin-linux += fifoWriter


obj-linux-fifoReader = $(obj-linux-fifo)
obj-linux-fifoReader += linux/tools/fifoReader.o
lib-linux-fifoReader += -lm
bin-linux += fifoReader

obj-linux-bootScenario = $(obj-linux-fifo)
obj-linux-bootScenario += linux/tools/bootScenario.o
lib-linux-bootScenario += -lm
bin-linux += bootScenario