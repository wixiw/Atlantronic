/*
 * boot_signals.h
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#ifndef BOOT_SIGNALS_H_
#define BOOT_SIGNALS_H_

#include "Signal.h"

extern Signal control_boot_signal;
extern Signal detection_boot_signal;
extern Signal fault_boot_signal;
extern Signal usb_boot_signal;

#define BOOT_SIGNAL_NB 4

extern Signal* boot_signals[BOOT_SIGNAL_NB];

#endif /* BOOT_SIGNALS_H_ */
