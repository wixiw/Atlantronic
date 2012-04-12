/*
 * FaulhaberStates.h
 *
 *  Created on: Apr 12, 2012
 *      Author: ard
 */

#ifndef FAULHABERSTATES_H_
#define FAULHABERSTATES_H_

namespace arp_hml
{
        enum HomingState
        {
            ASK_CONFIGURE_EDGE,
            WAIT_CONFIGURE_EDGE,
            ASK_CONFIGURE_SWITCH,
            WAIT_CONFIGURE_SWITCH,
            ASK_CONFIGURE_SPEED,
            WAIT_CONFIGURE_SPEED,
            ASK_HOMING,
            WAIT_HOMING,
            HOMING_DONE,
            NOT_IN_HOMNG_MODE
        };
}

#endif /* FAULHABERSTATES_H_ */
