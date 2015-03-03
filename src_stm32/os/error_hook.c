/*
 * error_hook.c
 *
 *  Created on: Feb 26, 2015
 *      Author: willy
 */

#include "error_hook.h"
#include "os.h"
#include "module.h"

#define PANIC_STACK_OVEFLOW   2
#define PANIC_MALLOC_FAILED   3
#define PANIC_ASSERT		  4

__OPTIMIZE_ZERO__  void kernel_panic(uint8_t err)
{
	UNUSED(err);
	taskDISABLE_INTERRUPTS();
	ardPanicFromIsr();

}

void vApplicationMallocFailedHook( void )
{
	taskDISABLE_INTERRUPTS();
	kernel_panic(PANIC_MALLOC_FAILED);
}

__OPTIMIZE_ZERO__ void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	UNUSED(pcTaskName);
	UNUSED(pxTask);

	taskDISABLE_INTERRUPTS();
	kernel_panic(PANIC_STACK_OVEFLOW);
}

void ardAssert(int cond)
{
	if( cond == 0 )
	{
		kernel_panic(PANIC_ASSERT);
	}
}


void ardPanicFromIsr()
{
	while(1)
	{
		//setLed(LED_RED);
	}
}
