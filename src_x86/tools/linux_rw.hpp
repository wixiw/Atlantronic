/*
 * linux_rw.hpp
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 */

#ifndef LINUX_RW_HPP_
#define LINUX_RW_HPP_

#ifndef LINUX
#define LINUX
#endif

#include "com/stack_com/IpcMsg.hpp"
#include <sys/stat.h>
#include <fcntl.h>

void displayMsgSizes();
void sendMsg(arp_stm32::IpcMsg& msg, int writeFd);
void sendMsgWithPause(arp_stm32::IpcMsg& msg, int writeFd);
bool receiveMessage(arp_stm32::Datagram& dtg, int readFd);
int openFd( int argc, char *argv[], mode_t mode);


#endif /* LINUX_RW_HPP_ */
