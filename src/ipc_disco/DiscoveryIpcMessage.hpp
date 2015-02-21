/*
 * DiscoveryIpcMessage.hpp
 *
 *  Created on: Feb 20, 2015
 *      Author: robot
 */

#ifndef DISCOVERYIPCMESSAGE_HPP_
#define DISCOVERYIPCMESSAGE_HPP_

#ifdef LINUX
#include "ipc_disco/MessagePrinter.hpp"
#endif

#include "ipc_disco/DiscoveryIpcTypes.h"

#include "ipc_disco/ConfigurationMsg.hpp"
#include "ipc_disco/EventMessage.hpp"
#include "ipc_disco/FaultMessage.hpp"
#include "ipc_disco/GyroMsg.hpp"
#include "ipc_disco/HokuyoMessage.hpp"
#include "ipc_disco/LogMessage.hpp"
#include "ipc_disco/OpponentListMsg.hpp"
#include "ipc_disco/RawMessage.hpp"
#include "ipc_disco/StatusMessage.hpp"
#include "ipc_disco/VersionMessage.hpp"
#include "ipc_disco/X86CmdMsg.hpp"


#endif /* DISCOVERYIPCMESSAGE_HPP_ */
