#ifndef FAULT_H
#define FAULT_H

//! @file fault.h
//! @brief Gestion des defauts
//! @author Atlantronic

#include <stdint.h>
#include "kernel/error_codes.h"

#define FAULT_CLEAR            0x00
#define FAULT_ACTIVE           0x01

struct fault_status
{
	uint32_t state; //!< dernier bit : actif ou non. state >> 1 : nombre de defauts de ce type depuis le debut
	uint64_t time;  //!< date du dernier defaut de ce type
} __attribute__((packed));

//!< monter / descendre un defaut
void fault(enum fault id, unsigned char new_state);

//!< monter / descendre un defaut depuis une IT
long fault_from_isr(enum fault id, unsigned char new_state);

#endif