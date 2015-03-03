/*
 * error_hook.h
 *
 *  Created on: Feb 26, 2015
 *      Author: robot
 */

#ifndef ERROR_HOOK_H_
#define ERROR_HOOK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


void kernel_panic(uint8_t err) __attribute__((noreturn));

//This function may be used to create a kernel panic on a null condition
void ardAssert(int cond);
//same but from isr()
void ardPanicFromIsr() __attribute__((noreturn));

#ifdef __cplusplus
}
#endif



#endif /* ERROR_HOOK_H_ */
