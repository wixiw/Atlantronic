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

struct control_usb_data
{
	struct systime current_time;
	uint32_t match_time_elapsed; //in ms
	uint32_t match_time_togo;   //in ms
	int16_t raw_data_gyro;
	uint32_t gpio;
	float omega_gyro;
	float pos_theta_gyro_euler;
	float pos_theta_gyro_simpson;
	float vBat;
	float iPwm[4];
	uint8_t pumpState;
	uint8_t color;
	uint32_t power_state;
	struct dynamixel_usb_data dynamixel;
} __attribute__((packed));

#ifdef __cplusplus
}
#endif

#endif
