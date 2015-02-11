/*
 * x86_cmd.h
 *
 *  Created on: Feb 9, 2015
 *      Author: robot
 */

#ifndef X86_CMD_H_
#define X86_CMD_H_

#include <stdint.h>
#include "kernel/math/vect_plan.h"
#include "kernel/pump.h"
#include "kernel/driver/dynamixel.h"

#define X86_STATE_UNKNOWN 0
#define X86_STATE_READY 1

typedef struct {
	uint8_t power_request;  	//1 when power is requested, 0 otherwise
	VectPlan position ;			//position of the robot on the table
	uint8_t pumpCmd[PUMP_MAX];	//pump command in %
	uint8_t x86_state;			//see defines at top
	struct dynamixel_cmd_param dynamixel_cmd[NB_MAX_AX12+NB_MAX_RX24];
} __attribute__((packed)) x86_cmd ;


#endif /* X86_CMD_H_ */
