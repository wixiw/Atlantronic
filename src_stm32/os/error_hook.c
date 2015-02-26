/*
 * error_hook.c
 *
 *  Created on: Feb 26, 2015
 *      Author: willy
 */

#include "os.h"
#include "module.h"

__OPTIMIZE_ZERO__  void kernel_panic(uint8_t err)
{
	(void) err;
	while(1)
	{
		//setLed(LED_RED);
	}
}

void vApplicationMallocFailedHook( void )
{
	taskDISABLE_INTERRUPTS();
	kernel_panic(0);
}

__OPTIMIZE_ZERO__ void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	UNUSED(pcTaskName);
	UNUSED(pxTask);

	taskDISABLE_INTERRUPTS();
	kernel_panic(0);
}

