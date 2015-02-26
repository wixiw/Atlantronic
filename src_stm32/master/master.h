/*
 * master.h
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 *
 *      This file is the proxy of the master code driving the robot on the x86.
 *      It syncronize its state with the master (in both way stm32<=>x86), and drive everything on the robot on master order.
 */

#ifndef MASTER_H_
#define MASTER_H_

typedef enum
{
	MASTER_STATE_WAITING_COLOR_CHOICE,
	MASTER_STATE_WAIT_AU_UP,
	MASTER_STATE_UNCONFIGURED,
	MASTER_STATE_DEPLOYING,
	MASTER_STATE_WAITING_SELF_TEST_BEGIN,
	MASTER_STATE_SELF_TESTING,
	MASTER_STATE_PLACING_ROBOT_FOR_MATCH,
	MASTER_STATE_WAITING_MATCH_BEGIN,
	MASTER_STATE_MATCH_RUNNING,
	MASTER_STATE_MATCH_ENDED,
	MASTER_STATE_MAX,
} eMasterState;


#endif /* MASTER_H_ */
