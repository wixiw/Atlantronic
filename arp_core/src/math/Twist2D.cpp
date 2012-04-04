/*
 * Twist2D.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include <math/Twist2D.hpp>

using namespace arp_math;

Twist2D::Twist2D(Vector2 _vitesseTranslation, double _vitesseRotation):
        vitesseTranslation(_vitesseTranslation),
        vitesseRotation(_vitesseRotation){}


Twist2D::Twist2D(double _vx, double _vy, double _vh):
        vitesseTranslation(_vx, _vy),
        vitesseRotation(_vh){}

Twist2D::Twist2D(Vector3 T):
        vitesseTranslation(T[1], T[2]),
        vitesseRotation(T[0]){}

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

std::string Twist2D::toString() const
{
    std::ostringstream  s;
    s << "(" << vx() << "," << vy() << "," << vh() << ")";
    return s.str();
}

bool Twist2D::operator ==(Twist2D other) const
{
    //le test de translation est fait à la main sinon on perd le const... visiblement eigen ne note pas ses == const... c'est moche
    return (
            this->vh() == other.vh()
            && this->vx() == other.vx()
            && this->vy() == other.vy()
            );
}

Vector3 Twist2D::getTVector() const
{
    Vector3 T;
    T[0] = vh();
    T[1] = vx();
    T[2] = vy();
    return T;
}

Twist2D Twist2D::transport(Pose2D p) const
{
    Vector3 res =  p.inverse().getBigAdjoint()*getTVector();
    return Twist2D(res);
}

