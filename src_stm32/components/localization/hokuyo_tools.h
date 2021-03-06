#ifndef HOKUYO_TOOLS_H
#define HOKUYO_TOOLS_H

//! @file hokuyo_tools.h
//! @brief Hokuyo tools
//! @author Atlantronic

#include <stdint.h>
#include "components/localization/hokuyo.h"
#include "polyline.h"

void hokuyo_compute_xy(struct hokuyo_scan* scan, struct vect2 *pos);

int hokuyo_find_objects(struct hokuyo_scan* scan, struct vect2* hokuyo_pos, unsigned int size, struct polyline* obj, unsigned int obj_size);

#endif
