/*
 * match_time.h
 *
 *  Created on: Feb 26, 2015
 *      Author: robot
 */

#ifndef MATCH_TIME_H_
#define MATCH_TIME_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "os/systime.h"


// recuperation du temps depuis le debut du match
struct systime systick_get_match_time(void);

// récupération du tickcount de début de match
portTickType systick_get_match_begin_tickcount(void);

// enregistrement du temps du debut du match (si match non débuté)
void systick_start_match_from_isr(void);

// recuperation temps debut de match
struct systime systick_get_match_begin_date();

#ifdef __cplusplus
}
#endif


#endif /* MATCH_TIME_H_ */
