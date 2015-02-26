/*
 * match_time.c
 *
 *  Created on: Feb 26, 2015
 *      Author: robot
 */

#include "os/os.h"

static struct systime systick_time_start_match;
portTickType systick_tickcount_start_match;

struct systime systick_get_match_time()
{
	struct systime t;
	portENTER_CRITICAL();
	t = systick_get_time_from_isr();
	t = timediff(t, systick_time_start_match);
	portEXIT_CRITICAL();

	return t;
}

void systick_start_match_from_isr()
{
	if( systick_time_start_match.ms == 0 && systick_time_start_match.ns == 0)
	{
		systick_time_start_match = systick_get_time_from_isr();
		systick_tickcount_start_match = xTaskGetTickCountFromISR();
	}
}

struct systime systick_get_match_begin_date()
{
	return systick_time_start_match;
}

portTickType systick_get_match_begin_tickcount(void)
{
	return systick_tickcount_start_match;
}

