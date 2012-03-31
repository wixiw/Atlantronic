/*
 * Twist2D.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include <math/Twist2D.hpp>

using namespace arp_math;

Twist2D::Twist2D(Vector2 _vitesseTranslation, double _vitesseRotation)
: vitesseTranslation(_vitesseTranslation)
, vitesseRotation(_vitesseRotation)
{
    ;
}


Twist2D::Twist2D(double _vx, double _vy, double _vh)
: vitesseTranslation(_vx, _vy)
, vitesseRotation(_vh)
{
    ;
}

double Twist2D::vx() const
{
    return vitesseTranslation[0];
}

double Twist2D::vy() const
{
    return vitesseTranslation[1];
}

double Twist2D::vh() const
{
    return vitesseRotation;
}

double Twist2D::speedAngle() const
{
    return atan2(vitesseTranslation[1], vitesseTranslation[0]);
}

void Twist2D::vx(double _vx)
{
    vitesseTranslation[0] = _vx;
}

void Twist2D::vy(double _vy)
{
    vitesseTranslation[1] = _vy;
}

void Twist2D::vh(double _vh)
{
    vitesseRotation = _vh;
}

