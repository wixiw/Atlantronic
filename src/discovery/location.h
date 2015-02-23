#ifndef LOCATION_H
#define LOCATION_H

//! @file location.h
//! @brief Location
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect_plan.h"

#ifndef WEAK_LOCALIZATION
#include "kernel/asm/asm_base_func.h"
#define WEAK_LOCALIZATION __attribute__((weak, alias("nop_function") ))
#endif

VectPlan location_get_position() WEAK_LOCALIZATION;
void location_set_position(VectPlan pos) WEAK_LOCALIZATION;

#endif
