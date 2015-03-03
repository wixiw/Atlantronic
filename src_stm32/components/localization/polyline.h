#ifndef POLYLINE_H
#define POLYLINE_H

//! @file polyline.h
//! @brief Polyline
//! @author Atlantronic

#include <stdint.h>
#include "vect2.h"

struct polyline
{
	struct vect2* pt;
	int16_t size;
} __attribute__((packed));

#endif
