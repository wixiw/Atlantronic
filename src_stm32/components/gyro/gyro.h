#ifndef GYRO_H
#define GYRO_H


#ifndef WEAK_GYRO
#include "core/asm/asm_base_func.h"
#define WEAK_GYRO __attribute__((weak, alias("nop_function") ))
#endif

enum GyroState
{
	GYRO_STATE_DISCONNECTED = 0,
	GYRO_STATE_RUNNING,
};

//enum for gyro_cmd.calib_cmd
#define	GYRO_CMD_CALIBRATION_START 	1
#define	GYRO_CMD_CALIBRATION_STOP 	2
#define	GYRO_CMD_SET_POSITION 		3
#define	GYRO_CMD_CONFIGURE 			4

typedef struct {

	uint8_t cmd;
	float theta;
	float scale;
	float bias;
	float dead_zone;

} __attribute__((packed)) gyro_cmd ;

int16_t gyro_get_raw_data() WEAK_GYRO;
float gyro_get_omega() WEAK_GYRO;
float gyro_get_theta_euler() WEAK_GYRO;
float gyro_get_theta_simpson() WEAK_GYRO;
void gyro_set_theta(float theta) WEAK_GYRO;

void gyro_calibration_cmd(gyro_cmd const& cmd) WEAK_GYRO;

#endif
