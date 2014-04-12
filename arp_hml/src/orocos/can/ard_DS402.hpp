/*
 * arp_DS402.h
 *
 *  Created on: 12 mars 2011
 *      Author: ard
 */

#ifndef ARP_DS402_H_
#define ARP_DS402_H_

#include <iostream>

#define ENUM_LIST(name) \
            case name:\
                   return "name";\
            break;

struct ArdDs402
{
    enum enum_DS402_state
    {
      NotReadyToSwitchOn    = 0b0000000000000000,
      SwitchOnDisabled      = 0b0000000001000000,
      ReadyToSwitchOn       = 0b0000000000100001,
      SwitchedOn            = 0b0000000000100011,
      OperationEnable       = 0b0000000000100111,
      QuickStopActive       = 0b0000000000000111,
      FaultReactionActive   = 0b0000000000001111,
      Fault                 = 0b0000000000001000,
      UnknownDs402State     = 0b1111111111111111
    };

    static const uint16_t MaskNotReadyToSwitchOn    = 0b0000000001001111;
    static const uint16_t MaskSwitchOnDisabled      = 0b0000000001001111;
    static const uint16_t MaskReadyToSwitchOn       = 0b0000000001101111;
    static const uint16_t MaskSwitchedOn            = 0b0000000001101111;
    static const uint16_t MaskOperationEnable       = 0b0000000001101111;
    static const uint16_t MaskQuickStopActive       = 0b0000000001101111;
    static const uint16_t MaskFaultReactionActive   = 0b0000000001001111;
    static const uint16_t MaskFault                 = 0b0000000001001111;

    static enum_DS402_state getStateFromCanStatusWord( const uint16_t canObjectValue )
    {
        enum_DS402_state state = UnknownDs402State;

        if( (canObjectValue & ArdDs402::MaskNotReadyToSwitchOn) == ArdDs402::NotReadyToSwitchOn )
        {
            state = ArdDs402::NotReadyToSwitchOn;
        }
        else if( (canObjectValue & ArdDs402::MaskSwitchOnDisabled) == ArdDs402::SwitchOnDisabled )
        {
            state = ArdDs402::SwitchOnDisabled;
        }
        else if( (canObjectValue & ArdDs402::MaskReadyToSwitchOn) == ArdDs402::ReadyToSwitchOn )
        {
            state = ArdDs402::ReadyToSwitchOn;
        }
        else if( (canObjectValue & ArdDs402::MaskSwitchedOn) == ArdDs402::SwitchedOn )
        {
            state = ArdDs402::SwitchedOn;
        }
        else if( (canObjectValue & ArdDs402::MaskOperationEnable) == ArdDs402::OperationEnable )
        {
            state = ArdDs402::OperationEnable;
        }
        else if( (canObjectValue & ArdDs402::MaskQuickStopActive) == ArdDs402::QuickStopActive )
        {
            state = ArdDs402::QuickStopActive;
        }
        else if( (canObjectValue & ArdDs402::MaskFaultReactionActive) == ArdDs402::FaultReactionActive )
        {
            state = ArdDs402::FaultReactionActive;
        }
        else if( (canObjectValue & ArdDs402::MaskFault) == ArdDs402::Fault )
        {
            state = ArdDs402::Fault;
        }
        else
        {
            state = ArdDs402::UnknownDs402State;
        }

        return state;


    }

    static std::string toString(enum_DS402_state state)
    {
        switch(state)
        {
            ENUM_LIST(NotReadyToSwitchOn);
            ENUM_LIST(SwitchOnDisabled);
            ENUM_LIST(ReadyToSwitchOn);
            ENUM_LIST(SwitchedOn);
            ENUM_LIST(OperationEnable);
            ENUM_LIST(QuickStopActive);
            ENUM_LIST(FaultReactionActive);
            ENUM_LIST(Fault);
            ENUM_LIST(UnknownDs402State);
            default:
                    return"unknown";
        }
    }
};
#endif /* ARP_DS402_H_ */
