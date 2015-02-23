#define WEAK_USB
#include "kernel/driver/power.h"
#include "heartbeat.h"
#include "kernel/systick.h"

#define HEARTBEAT_TIMEOUT                 5000

static systime heartbeat_last_time;
static int heartbeat_disabled = 1;

void heartbeat_kick()
{
	heartbeat_last_time = systick_get_time();
}

void heartbeat_enable()
{
	heartbeat_disabled = 0;
}

void heartbeat_update()
{
	systime t = systick_get_time();
	if( (t - heartbeat_last_time).ms > HEARTBEAT_TIMEOUT && ! heartbeat_disabled)
	{
		power_set(POWER_OFF_HEARTBEAT);
	}
	else
	{
		power_clear(POWER_OFF_HEARTBEAT);
	}
}
