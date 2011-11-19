#ifndef TRAPEZE_H
#define TRAPEZE_H

//! @file trapeze.h
//! @brief Trapezoidal speed
//! @author Atlantronic

struct trapeze
{
	float a_max;
	float v_max;

	float s;
	float v;
};

void trapeze_apply(struct trapeze* t, float s);

void trapeze_set(struct trapeze* t, float v_max, float a_max);

void trapeze_reset(struct trapeze* t, float s, float v);

#endif