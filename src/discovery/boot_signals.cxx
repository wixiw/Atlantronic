/*
 * boot_signals.cxx
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#include "boot_signals.h"

Signal boot_signals[BOOT_ID_SIZE];
uint8_t boot_signals_config;

void start_module(BootModuleId id)
{
	boot_signals[id].set();
}

void wait_start_signal(BootModuleId id)
{
	boot_signals[id].wait();
}

void modules_set_start_config(uint8_t config)
{
	boot_signals_config = config;
}

void start_all_modules()
{
	for( int id = 0 ; id < BOOT_ID_SIZE ; id++ )
	{
		//check that bit at position "id" is set
		if( boot_signals_config & (1 << id))
		{
			boot_signals[id].set();
		}
	}
}

