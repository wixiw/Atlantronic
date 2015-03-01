/*
 * dynamixel_types.h
 *
 *  Created on: Mar 2, 2015
 *      Author: robot
 */

#ifndef DYNAMIXEL_TYPES_H_
#define DYNAMIXEL_TYPES_H_

#define NB_MAX_AX12 2
#define NB_MAX_RX24 2

enum
{
	DYNAMIXEL_CMD_SET_ID = 2,
	DYNAMIXEL_CMD_SET_BAUDRATE,
	DYNAMIXEL_CMD_SET_MANAGER_BAUDRATE,
	DYNAMIXEL_CMD_SET_GOAL_POSITION,
	DYNAMIXEL_CMD_SET_SPEED,
	DYNAMIXEL_CMD_SET_MAX_TORQUE,
	DYNAMIXEL_CMD_SET_TARGET_REACHED_THRESHOLD,
	DYNAMIXEL_CMD_GET_POSITION,
	DYNAMIXEL_CMD_ENABLE_ENDLESS_TURN_MODE,
	DYNAMIXEL_CMD_DISABLE_ENDLESS_TURN_MODE,
};

struct dynamixel_cmd_param
{
	uint8_t type;           //!< type de dynamixel (ax12 ou rx24)
	uint8_t cmd_id;         //!< id de la commande
	uint8_t id;             //!< id du dynamixel
	uint8_t reserved;       //!< reserve
	float param;            //!< parametre
} __attribute((packed));

struct dynamixel_error
{
	//!< bit 7 à 4 : ERR_DYNAMIXEL_SEND_CHECK, ERR_DYNAMIXEL_PROTO ou ERR_DYNAMIXEL_CHECKSUM
	//!< bit 4 à 0 : erreur usart sur les 4 bits de poids faible
	uint8_t transmit_error;
	//!< erreur interne dynamixel (champ de bit)
	uint8_t internal_error;
} __attribute((packed));

struct dynamixel_usb_device_data
{
	uint16_t pos;           //!< position
	uint16_t flags;         //!< flags
	struct dynamixel_error error; //!< erreurs
} __attribute((packed));


struct dynamixel_usb_data
{
	struct dynamixel_usb_device_data ax12[NB_MAX_AX12];
	struct dynamixel_usb_device_data rx24[NB_MAX_RX24];

} __attribute((packed));


#endif /* DYNAMIXEL_TYPES_H_ */
