/*
 * boot_id.h
 *
 *  Created on: Feb 14, 2015
 *      Author: robot
 */

#ifndef BOOT_ID_H_
#define BOOT_ID_H_

typedef enum
{
	BOOT_ID_DETECTION,
	BOOT_ID_FAULT,
	BOOT_ID_DYNAMIXEL,
	BOOT_ID_HOKUYO,
	BOOT_ID_SIZE
} BootModuleId;



#endif /* BOOT_ID_H_ */
