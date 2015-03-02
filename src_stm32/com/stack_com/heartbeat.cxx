#define WEAK_USB
#include "heartbeat.h"
#include "os/systick.h"
#include "os/rcc.h"

static systime heartbeat_last_time;
static uint32_t timeoutInMs = 0;

extern void hearbeat_lost_CB();

void heartbeat_kick()
{
	heartbeat_last_time = systick_get_time();
}

void heartbeat_setTimeout(uint32_t new_timeoutInMs)
{
	timeoutInMs = new_timeoutInMs;
	heartbeat_kick();
}

void heartbeat_update()
{
	//Si le timeout est mis à 0 il n'y a pas de verification.
	if( timeoutInMs )
	{
		systime t = systick_get_time();

		if( timeoutInMs < (t - heartbeat_last_time).ms )
		{
			hearbeat_lost_CB();
		}
	}
}
