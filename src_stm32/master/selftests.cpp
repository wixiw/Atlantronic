/*
 * selftests.cpp
 *
 *  Created on: Mar 1, 2015
 *      Author: willy
 */

#include "selftests.hpp"
#include "components/pump/pump.h"
#include "components/dynamixel/dynamixel.h"
#include "os/os.h"

static bool selfTestFinished = false;

void setDefaultCmd()
{
	for(int i = 0; i < PUMP_MAX; i++)
	{
		pump[i].set(0.00);
	}

	ax12.set_goal_position(2, 0.0);
	ax12.set_goal_position(3, 0.0);
	rx24.set_goal_position(2, 0.0);
}

void selftests_run()
{
	//TODO
//	for(int i = 0; i < PUMP_MAX; i++)
//	{
//		pump[i].set(0.40);
//	}
//
//	ax12.set_goal_position(2, 1.0);
//	ax12.set_goal_position(3, 1.0);
//	rx24.set_goal_position(2, 1.0);

	vTaskDelay(ms_to_tick(1000));

	//Return to idle state.
	//TODO setDefaultCmd();

	selfTestFinished = true;
}

bool isSelftestFinished()
{
	return selfTestFinished;
}


