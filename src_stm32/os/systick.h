#ifndef SYSTICK_H
#define SYSTICK_H

//! @file systick.h
//! @brief Time module
//! @author Atlantronic
//! @author Willy
#include <stdint.h>
#include "systime.h"

#ifdef __cplusplus
extern "C" {
#endif

//!< recuperation du temps depuis le demarrage
struct systime systick_get_time(void);

//!< recuperation du temps depuis le demarrage (depuis une IT)
struct systime systick_get_time_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif
