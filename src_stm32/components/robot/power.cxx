#define WEAK_POWER

#include "power.h"
#include "core/gpio.h"
#include "os/module.h"
#include "components/log/log.h"
#include "components/pump/pwm.h"

int power_state = POWER_OFF;

void power_set(int powerEventMask)
{
	int old_state = power_state;
	power_state |= powerEventMask;
	if( power_state )
	{
		pwm_disable();
		if( power_state & ~POWER_OFF_HEARTBEAT )
		{
			gpio_power_off();
		}
	}

	int diff = power_state ^ old_state;
	if( diff & POWER_OFF_UNDERVOLTAGE )
	{
		log(LOG_ERROR, "power off - undervoltage");
	}
	if( diff & POWER_OFF_END_MATCH )
	{
		log(LOG_INFO, "power off - end match");
	}
	if( diff & POWER_OFF )
	{
		log(LOG_INFO, "power off");
	}
	if( diff & POWER_OFF_AU )
	{
		log(LOG_ERROR, "power off - AU");
	}
	if( diff & POWER_OFF_HEARTBEAT)
	{
		log(LOG_ERROR, "power off - HeartBeat");
	}
}

void power_clear(int powerEventMask)
{
	int old_state = power_state;
	power_state &= ~powerEventMask;

	int diff = powerEventMask & old_state;
	if( diff & POWER_OFF_UNDERVOLTAGE )
	{
		log(LOG_INFO, "power clear - undervoltage");
	}
	if( diff & POWER_OFF_END_MATCH )
	{
		log(LOG_INFO, "power clear - end match");
	}
	if( diff & POWER_OFF )
	{
		log(LOG_INFO, "power clear - off");
	}
	if( diff & POWER_OFF_AU )
	{
		log(LOG_INFO, "power clear - AU");
	}
	if( diff & POWER_OFF_HEARTBEAT)
	{
		log(LOG_ERROR, "power clear - HeartBeat");
	}

	if( ! (power_state & ~POWER_OFF_HEARTBEAT) )
	{
		gpio_power_on();
	}

	if( ! power_state )
	{
		pwm_enable();
		gpio_power_on();
		if(old_state != power_state)
		{
			log(LOG_INFO, "power on");
		}
	}
}

bool power_isEmergencyStopFired()
{
	return power_state & POWER_OFF_AU;
}
