/*
 * boot_signals.cxx
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#include "boot_signals.h"

Signal control_boot_signal;
Signal detection_boot_signal;
Signal fault_boot_signal;
Signal usb_boot_signal;
Signal dynamixel_boot_signal;
Signal hokuyo_boot_signal;

//Use this for debug purposes, this signal is never set.
Signal no_boot_signal;


Signal* boot_signals[BOOT_SIGNAL_NB] = {
		&usb_boot_signal,
		&fault_boot_signal,
		&control_boot_signal,
		&detection_boot_signal,
		&dynamixel_boot_signal,
		&hokuyo_boot_signal
};

