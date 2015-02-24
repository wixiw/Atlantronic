//! @file main.c
//! @brief Programme principal
//! @author Atlantronic

#include "core/module.h"
#include "gpio.h"
#include <stdint.h>
#include "os/os.h"

//! pour ne pas confondre avec le main de la libc newlib
void __main() __attribute__((noreturn));
void kernel_panic(uint8_t err) __attribute__((noreturn));

void kernel_panic(uint8_t err)
{
	UNUSED(err);
	while(1)
	{
		//setLed(LED_RED);
	}
}

void __main()
{
	uint8_t err = initModules();

	// on n'arrive normalement jamais ici sur la cible
	kernel_panic(err);
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
