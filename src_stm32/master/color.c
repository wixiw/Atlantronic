/*
 * color.c
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 */

#include "color.h"

static eMatchColor color;

eMatchColor getColor()
{
	return color;
}

void setColor(eMatchColor newColor)
{
	color = newColor;
}

