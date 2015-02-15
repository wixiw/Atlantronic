#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/math/vect_plan.h"
#include "kernel/math/matrix_homogeneous.h"
#include "kernel/driver/encoder.h"
#include "kernel/driver/dynamixel.h"

#ifdef __cplusplus
extern "C" {
#endif

//! période de la tache de controle en ms
#define EPSILON                                 1e-4

void set_control_period(uint8_t periodInMs);

struct control_usb_data
{
	struct systime current_time;
	int16_t raw_data_gyro;
	uint16_t encoder[ENCODER_MAX];
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
