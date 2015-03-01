/*
 * boot_signals.h
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#ifndef BOOT_SIGNALS_H_
#define BOOT_SIGNALS_H_

#include "os/Signal.h"
#include "boot_id.h"

#ifdef __cplusplus
extern "C" {
#endif

//Call this inside a module when ready to start
void wait_start_signal(BootModuleId id);

//Call this in your startup manager to send the boot signal for a defined module
void start_module(BootModuleId id);

//Call this to configure which module is started with start_all_modules
void modules_set_start_config(uint8_t config);

//Call this in your startup manager to start all configured modules
void start_optionnal_modules();

//Call this to sleep infinitly for debug purposes
void sleep_and_never_wakeup();

#ifdef __cplusplus
}
#endif

#endif /* BOOT_SIGNALS_H_ */
