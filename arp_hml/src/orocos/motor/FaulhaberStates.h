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
            ASK_CONFIGURE_SWITCH1,
            WAIT_CONFIGURE_SWITCH1,
            ASK_CONFIGURE_SWITCH2,
            WAIT_CONFIGURE_SWITCH2,
            ASK_CONFIGURE_SWITCH3,
            WAIT_CONFIGURE_SWITCH3,
            ASK_CONFIGURE_SPEED,
            WAIT_CONFIGURE_SPEED,
            ASK_HOMING,
            WAIT_HOMING,
            SET_NULL_SPEED,
            HOMING_DONE,
            NOT_IN_HOMNG_MODE
        };
}

#endif /* FAULHABERSTATES_H_ */
