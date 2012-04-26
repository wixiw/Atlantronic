/*
 * Twist2D.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include <math/Twist2D.hpp>

using namespace arp_math;

std::ostream &operator<<( std::ostream &flux, arp_math::Twist2D const& t)
{
    return flux << t.toString();
}

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

double& Twist2D::vxRef()
{
    return vitesseTranslation[0];
}

double& Twist2D::vyRef()
{
    return vitesseTranslation[1];
}

double& Twist2D::vhRef()
{
    return vitesseRotation;
}

double Twist2D::speedAngle() const
{
    return atan2(vitesseTranslation[1], vitesseTranslation[0]);
}

double Twist2D::speedNorm() const
{
    return vitesseTranslation.norm();
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

Twist2D Twist2D::operator+(const Twist2D& b) const
{
    return Twist2D( vx()+b.vx(), vy()+b.vy() , vh()+b.vh() );
}

Twist2D Twist2D::operator-(const Twist2D& b) const
{
    return Twist2D( vx()-b.vx(), vy()-b.vy() , vh()-b.vh() );
}

Twist2D Twist2D::operator*(const double& scalaire) const
{
    return Twist2D( vx()*scalaire, vy()*scalaire , vh()*scalaire );
}

Twist2D Twist2D::operator/(const double& scalaire) const
{
    return Twist2D( vx()/scalaire, vy()/scalaire , vh()/scalaire );
}

Twist2D& Twist2D::operator+=(const Twist2D& _other)
{
    vx(vx() + _other.vx());
    vy(vy() + _other.vy());
    vh(vh() + _other.vh());

    return *this;
}

Twist2D& Twist2D::operator-=(const Twist2D& _other)
{
    vx(vx() - _other.vx());
    vy(vy() - _other.vy());
    vh(vh() - _other.vh());

    return *this;
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

bool Twist2D::operator !=(Twist2D other) const
{
    //le test de translation est fait à la main sinon on perd le const... visiblement eigen ne note pas ses == const... c'est moche
    return (
            this->vh() != other.vh()
            || this->vx() != other.vx()
            || this->vy() != other.vy()
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

double Twist2D::distanceTo(Twist2D other, Vector3 coef) const
{
    double dh2 = (vh()-other.vh())*(vh()-other.vh());
    double dx2 = (vx()-other.vx())*(vx()-other.vx());
    double dy2 = (vy()-other.vy())*(vy()-other.vy());
    return sqrt(coef(0)*coef(0)*dh2 + coef(1)*coef(1)*dx2 + coef(2)*coef(2)*dy2);
}

void Twist2D::limitFirstDerivate(Twist2D lastTwist, Vector3 limits, double period)
{
    vx( firstDerivateLimitation(vx(),lastTwist.vx(), period, -fabs(limits[0]), fabs(limits[0])) );
    vy( firstDerivateLimitation(vy(),lastTwist.vy(), period, -fabs(limits[1]), fabs(limits[1])) );
    vh( firstDerivateLimitation(vh(),lastTwist.vh(), period, -fabs(limits[2]), fabs(limits[2])) );
}

Twist2D Twist2D::createFromPolar(double normV, double angV, double vh)
{
    Twist2D t;
    t.vx(normV*cos(angV));
    t.vy(normV*sin(angV));
    t.vh(vh);
    return t;
}
