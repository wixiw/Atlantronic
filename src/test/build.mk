#ipc folder
obj-test-link_test += ipc/Datagram.o
obj-test-link_test += ipc/IpcHeader.o
obj-test-link_test += ipc/IpcMsg.o

#ipc_disco folder
obj-test-link_test += ipc_disco/RawMessage.o


obj-test-link_test  += test/main.o
bin-test += link_test
