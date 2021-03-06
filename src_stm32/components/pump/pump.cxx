#define WEAK_PUMP
#include "pump.h"
#include "pwm.h"
#include "os/module.h"
#include "core/adc.h"
#include "components/log/log.h"

// pompe 12V
#define PUMP_VMAX                          12
#define PUMP_BLOCKED_MIN_STDDEV2       0.003f
#define PUMP_UNBLOCKED_MAX_STDDEV2     0.002f
#define PUMP_CURRENT_PIC_DELAY_MS         200

Pump pump[PUMP_MAX] =
{
	Pump(PWM_1),
	Pump(PWM_2),
	Pump(PWM_3),
	Pump(PWM_4)
};

void pumps_update()
{
	for(int i = 0; i < PUMP_MAX; i++)
	{
		pump[i].update();
	}

}

uint32_t pumps_get_state()
{
	uint32_t res = 0;

	for(int i = 0; i < PUMP_MAX; i++)
	{
		res |= (pump[i].isBlocked() << i);
	}

	return res;
}

Pump::Pump(uint8_t Pwm_id)
{
	currentId = 0;
	pwm_id = Pwm_id;
	val = 0;
	pumpBlocked = false;
	for(int i = 0; i < PUMP_SAMPLE_STDDEV; i++)
	{
		current[i] = 0;
	}
	stdDev2 = 0;
}

void Pump::set(float percent)
{
	if(percent < 0)
	{
		val = 0;
	}
	else if ( percent > 1)
	{
		if( val == 0 )
		{
			startTime = systick_get_time();
		}
		val = 1;
	}
	else
	{
		if( val == 0 )
		{
			startTime = systick_get_time();
		}
		val = percent;
	}
}

void Pump::update()
{
	float mean = 0;
	float mean2 = 0;

	current[currentId] = adc_filtered_data.i[pwm_id];
	currentId = (currentId + 1) % PUMP_SAMPLE_STDDEV;
	for(int i = 0; i < PUMP_SAMPLE_STDDEV; i++)
	{
		mean += current[i];
		mean2 += current[i] * current[i];
	}
	mean = mean / PUMP_SAMPLE_STDDEV;
	mean2 = mean2 / PUMP_SAMPLE_STDDEV;
	stdDev2 = mean2 - mean * mean;

	systime dt = systick_get_time() - startTime;

	// pas de detection de bloquage si on est a moins de 50%
	if(val < 0.5f)
	{
		stdDev2 = 0;
	}

	if( dt.ms > PUMP_CURRENT_PIC_DELAY_MS && stdDev2 * val > PUMP_BLOCKED_MIN_STDDEV2 )
	{
		if( ! pumpBlocked )
		{
			log_format(LOG_INFO, "pump %d blocked", pwm_id);
		}
		pumpBlocked = true;
	}
	else if ( stdDev2 * val < PUMP_UNBLOCKED_MAX_STDDEV2 )
	{
		if( pumpBlocked )
		{
			log_format(LOG_INFO, "pump %d unblocked", pwm_id);
		}
		pumpBlocked = false;
	}
	pwm_set(pwm_id, val * PUMP_VMAX / adc_filtered_data.vBat);
}
