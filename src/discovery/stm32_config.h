/*
 * x86_cmd.h
 *
 *  Created on: Feb 9, 2015
 *      Author: robot
 */

#ifndef STM32_CONFIG_H_
#define STM32_CONFIG_H_

#include <stdint.h>

typedef struct {

	uint32_t match_duration; //in ms

} __attribute__((packed)) stm32_config ;


#endif /* X86_CMD_H_ */
