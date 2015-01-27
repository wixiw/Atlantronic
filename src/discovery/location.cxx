//! @file location.c
//! @brief Location
//! @author Atlantronic

#include "location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/driver/usb.h"

static void location_cmd_set_position(void* arg);
VectPlan location_pos(0, 700, -M_PI/2);
VectPlan location_speed;

static int location_module_init()
{
	usb_add_cmd(USB_CMD_LOCATION_SET_POSITION, location_cmd_set_position);

	return 0;
};

module_init(location_module_init, INIT_LOCATION);


VectPlan location_get_position()
{
	VectPlan p;
	portENTER_CRITICAL();
	p = location_pos;
	portEXIT_CRITICAL();
	return p;
}

void location_set_position(VectPlan pos)
{
	portENTER_CRITICAL();
	location_pos = pos;
	portEXIT_CRITICAL();
}

static void location_cmd_set_position(void* arg)
{
	VectPlan* pos = (VectPlan*) arg;
	location_set_position(*pos);
}
