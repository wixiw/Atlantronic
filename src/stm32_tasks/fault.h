#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include "kernel/error_codes.h"

#define FAULT_CLEAR            0x00
#define FAULT_ACTIVE           0x01

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_FAULT
#include "kernel/asm/asm_base_func.h"
#define WEAK_FAULT __attribute__((weak, alias("nop_function") ))
#endif

struct fault_status
{
	uint32_t state;      //!< dernier bit : actif ou non. state >> 1 : nombre de defauts de ce type depuis le debut
	uint32_t time;       //!< date du dernier defaut de ce type (en ms)
	char debugtext[16];  //!< infos debug
} __attribute__((packed));

//!< monter / descendre un defaut
void fault(enum fault id, unsigned char new_state) WEAK_FAULT;

//!< monter / descendre un defaut depuis une IT
long fault_from_isr(enum fault id, unsigned char new_state, const char* debugText) WEAK_FAULT;

#ifdef __cplusplus
}
#endif

#endif
