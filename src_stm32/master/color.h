/*
 * color.h
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 *
 *
 */

#ifndef COLOR_H_
#define COLOR_H_

/**
 * The robot may start on 2 different colors. As a convention, we always have a prefered color
 * so we only encode the strategy on one side.
 * The other side is symetrized from the prefered color.
 * As color change every year, we use a "prefered color" and a "symetrized color".
 * the "unknown color" is used to tag the fact that the robot is not configured yer
 */

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	COLOR_PREF,
	COLOR_SYM,
	COLOR_UNKNOWN
} eMatchColor;

eMatchColor getColor();
void setColor(eMatchColor color);

#ifdef __cplusplus
}
#endif


#endif /* COLOR_H_ */
