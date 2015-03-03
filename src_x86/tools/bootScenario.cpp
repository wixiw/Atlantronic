/*
 * fifoWriter.cpp
 *
 *  Created on: Dec 28, 2014
 *      Author: willy
 */

#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include "com/stack_com/IpcHeader.hpp"
#include "com/msgs/DiscoveryIpcMessage.hpp"
#include "linux_rw.hpp"

using namespace std;
using namespace arp_stm32;

int main ( int argc, char *argv[] )
{
	int writeFd = openFd(argc, argv, O_WRONLY);

	displayMsgSizes();

	char line[100];
	cout << "hit enter to start..." << endl;
	std::cin.getline(line, 100);

//	EventMessage msgReboot(EVT_REBOOT);
//	cout << "Reboot request sended." << endl;
//	sendMsg(msgReboot, writeFd);
//	cout << "Waiting reboot for 3s..." << endl;
//	sleep(3);
//	do {writeFd = openFd(argc, argv, O_WRONLY);} while(writeFd <= 0);
//
//	cout << "hit enter to start..." << endl;
//	std::cin.getline(line, 100);

	ConfigurationMsg msgConfig;
	msgConfig.setMatchDuration(5.0);
	msgConfig.setHokuyoDebug();
	msgConfig.setControlPeriod(100);
	msgConfig.setHeartbeatTimeout(0);
	cout << "Config request sended." << endl;
	sendMsgWithPause(msgConfig, writeFd);

	EventMessage msgx86readyForMatch(EVT_X86_READY_FOR_MATCH);
	cout << "Ready for match sended." << endl;
	sendMsgWithPause(msgx86readyForMatch, writeFd);


	int i = 0;
	while (1) {
		X86CmdMsg msgCmd;
		msgCmd.powerOn(true);
		sendMsg(msgCmd, writeFd);

//		if (i % 10 == 0) {
//			EventMessage msgEvt(EVT_LIST_TASKS);
//			sendMsg(msgEvt, writeFd);
//		}
		i++;
		cout << "i=" << i << endl;
		usleep(10 * 1E3);
	}
}

