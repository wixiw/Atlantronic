#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include <stdint.h>
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "kernel/control/kinematics.h"

enum
{
	CAN_MOTOR_DRIVING1 = 0,
	CAN_MOTOR_STEERING1,
	CAN_MOTOR_DRIVING2,
	CAN_MOTOR_STEERING2,
	CAN_MOTOR_DRIVING3,
	CAN_MOTOR_STEERING3,
	CAN_MOTOR_MAX,
};

enum homing_status
{
	CAN_MOTOR_HOMING_NONE,
	CAN_MOTOR_HOMING_RUN_HP,
	CAN_MOTOR_HOMING_RUN_SHL,
	CAN_MOTOR_HOMING_RUN_SHA,
	CAN_MOTOR_HOMING_RUN_SHN,
	CAN_MOTOR_HOMING_RUN_HOSP,
	CAN_MOTOR_HOMING_RUN_GOHOSEQ,
	CAN_MOTOR_HOMING_RUNING_GOHOSEQ,
	CAN_MOTOR_HOMING_DONE,
};

class CanMotor : public CanopenNode
{
	public:
		CanMotor();

		const char* name;

		// donnees brutes
		uint16_t status_word;
		uint32_t raw_position; //!< position brute (en increments encodeurs)
		uint16_t current;
		Kinematics kinematics;
		enum homing_status homingStatus;

		//systime date;
		float inputGain;    //!< gain pour convertir la vitesse en unites moteurs (rpm)
		float outputGain;   //!< gain pour convertir la position en unites robot
		float positionOffset; //!< offset sur la position

		void update();
		void update_homing(float v);
		void update_state();
		void set_speed(float v);

		inline bool is_op_enable()
		{
			return (status_word & 0x6f) == 0x27;
		}
	protected:
		virtual void rx_pdo(struct can_msg *msg, int type);
};

extern CanMotor can_motor[CAN_MOTOR_MAX];

#endif
