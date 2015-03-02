#define WEAK_CONTROL

#include "os/module.h"
#include "core/adc.h"
#include "components/pump/pump.h"
#include "os/os.h"
#include "control.h"
#include "components/log/log.h"

#define CONTROL_STACK_SIZE       200

static uint8_t control_task_period = 10;

static void control_task(void* arg);

static int control_module_init()
{
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return MODULE_INIT_SUCCESS;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* /*arg*/)
{
	uint32_t wake_time = 0;

	while(1)
	{
		// mise a jour adc
		adc_update();

		//mise a jour des pompes
		pumps_update();

		vTaskDelayUntil(&wake_time, control_task_period);
	}
}

void set_control_period(uint8_t periodInMs)
{
	log_format(LOG_INFO, "control periode => %d ms", (int)periodInMs);
	control_task_period = periodInMs;
}
