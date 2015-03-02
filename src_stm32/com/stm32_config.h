/*
 * x86_cmd.h
 *
 *  Created on: Feb 9, 2015
 *      Author: robot
 */

#ifndef STM32_CONFIG_H_
#define STM32_CONFIG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {

	uint32_t match_duration; //in ms
	uint8_t start_module_flags;
	uint8_t control_task_period; //in ms
	uint8_t hearbeat_timeout; //in s

} __attribute__((packed)) stm32_config ;

#ifdef __cplusplus
}
#endif

#endif /* X86_CMD_H_ */
