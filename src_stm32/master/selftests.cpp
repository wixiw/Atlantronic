/*
 * selftests.cpp
 *
 *  Created on: Mar 1, 2015
 *      Author: willy
 */

#include "selftests.hpp"
#include "components/pump/pump.h"
#include "os/os.h"

static bool selfTestFinished = false;

void selftests_run()
{
	for(int i = 0; i < PUMP_MAX; i++)
	{
		pump[i].set(0.40);
	}

	vTaskDelay(ms_to_tick(1000));

	for(int i = 0; i < PUMP_MAX; i++)
	{
		pump[i].set(0.00);
	}

	selfTestFinished = true;
}

bool isSelftestFinished()
{
	return selfTestFinished;
}


