#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_USB
#include "core/asm/asm_base_func.h"
#define WEAK_USB __attribute__((weak, alias("nop_function") ))
#endif

void heartbeat_kick() WEAK_USB;
void heartbeat_enable() WEAK_USB;
void heartbeat_update() WEAK_USB;


#ifdef __cplusplus
}
#endif

#endif
