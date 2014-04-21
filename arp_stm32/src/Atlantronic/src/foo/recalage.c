#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/robot_parameters.h"
#include "kernel/driver/usb.h"
#include "gpio.h"
#include "foo/arm.h"
#include "location/location.h"
#include "pince.h"
#include "control/trajectory.h"

void recalage();
int recalage_module_init();

int recalage_module_init()
{
	usb_add_cmd(USB_CMD_RECALAGE, &recalage);

	return 0;
}

module_init(recalage_module_init, INIT_STRATEGY);

void recalage()
{
	log(LOG_INFO, "recalage...");

	// TODO faire un truc propre
	extern uint8_t gpio_recalage_done;
	gpio_recalage_done = 1;

	struct fx_vect_pos pos;

	pince_set_position(PINCE_CLOSE,PINCE_CLOSE);

	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-800 << 16, 800 << 16, -1 << 24);
	}
	else
	{
		location_set_position(800 << 16, 800 << 16, -1 << 24);
	}

	control_set_max_speed( (1 << 16) / 3, 1 << 16);
	trajectory_disable_hokuyo();
	trajectory_disable_static_check();
	control_disable_sick();

	trajectory_straight_to_wall();
	if( trajectory_wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location_get_position();
	location_set_position(pos.x, (1000 << 16) + PARAM_NP_X, -1 << 24);
	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(10));

	trajectory_straight( (225 << 16) + PARAM_NP_X);
	if( trajectory_wait( TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	if(getcolor() == COLOR_BLUE)
	{
		trajectory_rotate_to(0);
	}
	else
	{
		trajectory_rotate_to(1 << 25);
	}

	if( trajectory_wait( TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	trajectory_straight(-2000 << 16);
	if( trajectory_wait( TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	trajectory_straight_to_wall();
	if( trajectory_wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location_get_position();
	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-((1500 << 16) + PARAM_NP_X), pos.y, 0);
	}
	else
	{
		location_set_position(((1500 << 16) + PARAM_NP_X), pos.y, 1 << 25);
	}

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(10));

	trajectory_straight(200 << 16);
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	vTaskDelay(ms_to_tick(200));

	arm_set_tool_way(0);

	log(LOG_INFO, "recalage termine");

free:
	trajectory_free();
	trajectory_enable_hokuyo();
	trajectory_enable_static_check();
	control_enable_sick();
	control_set_max_speed(1 << 16, 1 << 16);
}