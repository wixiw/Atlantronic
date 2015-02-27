#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "os/systick.h"
#include "components/localization/vect_plan.h"
#include "components/dynamixel/dynamixel.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_CONTROL
#include "core/asm/asm_base_func.h"
#define WEAK_CONTROL __attribute__((weak, alias("nop_function") ))
#endif

//! période de la tache de controle en ms
#define EPSILON                                 1e-4

void set_control_period(uint8_t periodInMs) WEAK_CONTROL;

#ifdef __cplusplus
}
#endif

#endif
