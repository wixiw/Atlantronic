#ifndef DETECTION_H
#define DETECTION_H

//! @file detection.h
//! @brief Detection
//! @author Atlantronic

#include <stdint.h>
#include "vect_pos.h"
#include "vect_plan.h"

#define DETECTION_NUM_OBJECT         100

enum detection_type
{
	DETECTION_FULL,
	DETECTION_STATIC_OBJ,
	DETECTION_DYNAMIC_OBJ,
};

struct detection_object
{
	float x;
	float y;
	float size;
} __attribute__((packed));

typedef void (*detection_callback)(void);

void detection_register_callback(detection_callback callback);

//!< Calcule en fonction de la position (actuelle ou non) pos du robot le segment [a b] (repère table) tel
//!< que [a b] soit la projection - sur l'axe Y du repère robot - de l'obstacle le plus proche dans l'axe du robot.
//!< => dans le repère robot, a.x = b.x = distance avant collision (valeur retournée) et [a.y b.y] largeur de
//!< l'objet vue par le robot. Le robot ne doit pas dépasser le segment [a b]
//!<
//!< attention, prise de mutex sur l'ensemble des segments
//!<
//!< @return distance maximale d'avance avant collision
float detection_compute_front_object(enum detection_type type, const VectPlan* pos, const vect2* a, const vect2* b, float dist_min);

#endif
