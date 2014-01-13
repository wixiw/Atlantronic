/*
 * sdo_ard.h
 *
 *  Created on: 6 mars 2011
 *      Author: ard
 */

#ifndef ARD_CAN_TYPES_H_
#define ARD_CAN_TYPES_H_

#include <canfestival/canfestival.h>
#include "orocos/can/ard_DS402.hpp"

namespace arp_hml
{
typedef int nodeID_t;

enum eRunningState
{
    UNKNOWN, PREOP, OPERATIONAL, IDLE, UNCONNECTED
} ;

enum enum_DS301_nmtStateRequest
{
    StartNode           = NMT_Start_Node,
    StopNode            = NMT_Stop_Node,
    EnterPreOp          = NMT_Enter_PreOperational,
    ResetNode           = NMT_Reset_Node,
    ResetComunication   = NMT_Reset_Comunication,
    UnknownRequest      = -1
};

struct CanDicoEntry
{
    CanDicoEntry():
        nodeId(0xFF),
        index(0),
        subindex(0),
        value(0),
        dataType(0),
        size(0){}

    CanDicoEntry(int _nodeId, int _index, int _subindex, int _value, int _dataType, int _size):
        nodeId(_nodeId),
        index(_index),
        subindex(_subindex),
        value(_value),
        dataType(_dataType),
        size(_size){}


    nodeID_t nodeId;
    int index;
    int subindex;
    int value;
    int dataType;
    int size;
};
}

#endif /* ARD_CAN_TYPES_H_ */
