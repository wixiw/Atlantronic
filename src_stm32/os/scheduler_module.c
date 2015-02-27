/*
 * scheduler.c
 *
 *  Created on: Feb 26, 2015
 *      Author: willy
 */

#include "module.h"
#include "components/log/log.h"
#include "core/cpu/cpu.h"
#include "os/os.h"

static int systick_module_init()
{
	//log(LOG_INFO, "Lancement de l'ordonanceur");

	// 4 bits for pre-emption priority, 0 bit sub priority
	SCB->AIRCR = 0x05FA0300;

	vTaskStartScheduler();

	return ERR_SYSTICK;
}

module_init(systick_module_init, INIT_SYSTICK);



