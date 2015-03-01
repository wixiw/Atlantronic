/*
 * master.h
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 *
 *      This file is the proxy of the master code driving the robot on the x86.
 *      It syncronize its state with the master (in both way stm32<=>x86), and drive everything on the robot on master order.
 */

#ifndef MASTER_H_
#define MASTER_H_

#include "os/systime.h"
#include "components/dynamixel/dynamixel_types.h"
#include "com/stack_com/Datagram.hpp"

typedef enum
{
	MASTER_STATE_WAITING_START_IN,
	MASTER_STATE_WAITING_COLOR_CHOICE,
	MASTER_STATE_WAIT_AU_UP,
	MASTER_STATE_STM32_SELF_TESTS,
	MASTER_STATE_WAITING_SELF_TEST_X86,
	MASTER_STATE_X86_SELF_TESTING,
	MASTER_STATE_WAITING_START_PLUG,
	MASTER_STATE_WAITING_MATCH_BEGIN,
	MASTER_STATE_MATCH_RUNNING,
	MASTER_STATE_MATCH_ENDED,
	MASTER_STATE_MAX,
} eMasterState;

struct control_usb_data
{
	struct systime current_time;
	uint32_t match_time_elapsed; //in ms
	uint32_t match_time_togo;   //in ms
	int16_t raw_data_gyro;
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


void setX86ReadyForMatch();
void master_command(arp_stm32::Datagram& dtg);
void master_configuration(arp_stm32::Datagram& dtg);

#endif /* MASTER_H_ */
