//! @file location.c
//! @brief Location
//! @author Atlantronic

#define WEAK_LOCALIZATION

#include "location.h"
#include "os/module.h"
#include "os/os.h"

VectPlan location_pos(0, 700, -M_PI/2);
VectPlan location_speed;



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

