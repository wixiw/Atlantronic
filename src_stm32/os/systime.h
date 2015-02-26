/*
 * systime.h
 *
 *  Created on: Feb 26, 2015
 *      Author: robot
 */

#ifndef SYSTIME_H_
#define SYSTIME_H_

#ifdef __cplusplus
extern "C" {
#endif

//! structure pour le temps
//! on utilise des ms et ns au lieu des secondes et ns pour simplifier le code
struct systime
{
	uint32_t ms; //!< temps en ms
	int32_t ns; //!< reste du temps en ns
};

//!< soustraction entre 2 systime
struct systime timediff(const struct systime t2, const struct systime t1);

//!< addition entre 2 systime
struct systime timeadd(const struct systime t1, const struct systime t2);

#ifdef __cplusplus
}

static inline struct systime operator +(const struct systime t1, const struct systime t2)
{
	return timeadd(t1, t2);
}

static inline struct systime operator -(const struct systime t2, const struct systime t1)
{
	return timediff(t2, t1);
}
#endif

#endif /* SYSTIME_H_ */
