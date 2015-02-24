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

#include "com/msgs/DiscoveryIpcTypes.h"

#include "com/msgs/ConfigurationMsg.hpp"
#include "com/msgs/EventMessage.hpp"
#include "com/msgs/FaultMessage.hpp"
#include "com/msgs/GyroMsg.hpp"
#include "com/msgs/HokuyoMessage.hpp"
#include "com/msgs/LogMessage.hpp"
#include "com/msgs/OpponentListMsg.hpp"
#include "com/msgs/RawMessage.hpp"
#include "com/msgs/StatusMessage.hpp"
#include "com/msgs/VersionMessage.hpp"
#include "com/msgs/X86CmdMsg.hpp"


#endif /* DISCOVERYIPCMESSAGE_HPP_ */
