#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "control.h"
#include "location.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/gyro.h"
#include "kernel/driver/adc.h"
#include "kernel/driver/power.h"
#include "kernel/heartbeat.h"
#include "gpio.h"
#include "fault.h"
#include "kernel/pump.h"
#include "boot_signals.h"
#include "ipc_disco/StatusMessage.hpp"
#include "kernel/driver/usb/ArdCom.h"
#include "stm32_tasks/end.h"

using namespace arp_stm32;

#define CONTROL_STACK_SIZE       1024

static struct control_usb_data control_usb_data;
static uint8_t control_task_period = 10;

static void control_task(void* arg);

static int control_module_init()
{
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* /*arg*/)
{
	uint32_t wake_time = 0;

	wait_start_signal(BOOT_ID_CONTROL);

	while(1)
	{
		// mise a jour adc
		adc_update();

		// mise a jour heartbeat
		heartbeat_update();

		control_usb_data.current_time = systick_get_time();
		control_usb_data.match_time_elapsed = systick_get_match_time().ms;
		control_usb_data.match_time_togo = end_get_match_time_togo();
		control_usb_data.raw_data_gyro = gyro_get_raw_data();
        control_usb_data.encoder[ENCODER_1] = encoder_get(ENCODER_1);
        control_usb_data.encoder[ENCODER_2] = encoder_get(ENCODER_2);
        control_usb_data.encoder[ENCODER_3] = encoder_get(ENCODER_3);
        control_usb_data.gpio = gpio_get_state();
		control_usb_data.omega_gyro = gyro_get_omega();
		control_usb_data.pos_theta_gyro_euler = gyro_get_theta_euler();
		control_usb_data.pos_theta_gyro_simpson = gyro_get_theta_simpson();
		control_usb_data.vBat = adc_filtered_data.vBat;
		control_usb_data.iPwm[0] = adc_filtered_data.i[0];
		control_usb_data.iPwm[1] = adc_filtered_data.i[1];
		control_usb_data.iPwm[2] = adc_filtered_data.i[2];
		control_usb_data.iPwm[3] = adc_filtered_data.i[3];
		control_usb_data.pumpState = pump_update();
		control_usb_data.color = getcolor();
		control_usb_data.power_state = power_get();
		dynamixel_update_usb_data(&control_usb_data.dynamixel);

		//Send message on usb com stack
		StatusMessage msg(control_usb_data);
		ArdCom::getInstance().send(msg);

		vTaskDelayUntil(&wake_time, control_task_period);
	}
}

void set_control_period(uint8_t periodInMs)
{
	log_format(LOG_INFO, "control periode => %d ms", (int)periodInMs);
	control_task_period = periodInMs;
}
