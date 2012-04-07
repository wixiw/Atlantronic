/*
 * Pose2D.cpp
 *
 *  Created on: 10 sept. 2010
 *      Author: boris
 */

#include "Pose2D.hpp"
#include <iostream>

using namespace arp_math;

Pose2D::Pose2D(Vector2 _positionTranslation, Rotation2 _positionRotation)
: positionTranslation(_positionTranslation)
, positionRotation(_positionRotation)
{
    ;
}

Pose2D::Pose2D(double _x, double _y, double _h)
: positionTranslation(Vector2(_x, _y))
, positionRotation(Rotation2(_h))
{
    ;
}


Vector2 Pose2D::translation() const
{
    return positionTranslation;
}

Rotation2 Pose2D::orientation() const
{
    return positionRotation;
}

double Pose2D::x() const
{
    return positionTranslation(0);
}

double Pose2D::y() const
{
    return positionTranslation(1);
}

double Pose2D::h() const
{
    return positionRotation.angle();
}

//pour les typkits Orocos il faut un getter avec reference

double& Pose2D::xRef()
{
    return positionTranslation(0);
}

double& Pose2D::yRef()
{
    return positionTranslation(1);
}

double& Pose2D::hRef()
{
    return positionRotation.angle();
}

double Pose2D::angle() const
{
    return positionRotation.angle();
}

Eigen::Matrix<double,3,3> Pose2D::getDisplacement2Matrix() const
{
	Eigen::Matrix<double,3,3> mat = Eigen::Matrix<double,3,3>::Identity();
	mat.topLeftCorner(2,2) = this->orientation().toRotationMatrix();
	mat.topRightCorner(2,1) = this->translation();
	return mat;
}

void Pose2D::translation(Vector2 _positionTranslation)
{
    positionTranslation = _positionTranslation;
}

void Pose2D::orientation(Rotation2 _positionRotation)
{
    positionRotation = _positionRotation;
}

void Pose2D::x(double _x)
{
    positionTranslation(0) = _x;
}

void Pose2D::y(double _y)
{
    positionTranslation(1) = _y;
}

void Pose2D::h(double _heading)
{
    positionRotation = Rotation2(_heading);
}

void Pose2D::angle(double _heading)
{
    positionRotation = Rotation2(_heading);
}

Pose2D Pose2D::inverse() const
{
    Pose2D p;
    p.translation(-this->translation());
    p.orientation(this->orientation().inverse());
    return p;
}

std::string Pose2D::toString() const
{
    std::ostringstream  s;
    s << "(" << x() << "," << y() << "," << h() << ")";
    return s.str();
}

bool Pose2D::operator ==(Pose2D other) const
{
    //le test de translation est fait à la main sinon on perd le const... visiblement eigen ne note pas ses == const... c'est moche
    return (
            this->h() == other.h()
            && this->translation()[0] == other.translation()[0]
            && this->translation()[1] == other.translation()[1]
            );
}

BigAdjoint2 Pose2D::getBigAdjoint() const
{
    BigAdjoint2 Ad = BigAdjoint2::Identity();
    Eigen::Matrix<double,2,2> R = this->orientation().toRotationMatrix();
    Ad(0,0) = 1;
    Ad(1,0) = y();
    Ad(2,0) = -x();
    Ad.bottomRightCorner(2,2) = R;
    return Ad;
}

double Pose2D::distanceTo(Pose2D pose) const
{
    double dx = pose.x() - x();
    double dy = pose.y() - y();
    return sqrt(dx * dx + dy * dy);
}

double Pose2D::angleTo(Pose2D pose) const
{
    return betweenMinusPiAndPlusPi(pose.h() - h());
}


Pose2D Pose2D::operator+(const Pose2D& b)
{
	return Pose2D( this->translation() + b.translation() , this->angle() + b.angle() );
}

Pose2D Pose2D::operator-(const Pose2D& b)
{
	return Pose2D( this->translation() - b.translation() , this->angle() - b.angle() );
}

std::ostream& operator <<(std::ostream& os, Pose2D _pose)
{
  os << _pose.translation().transpose() << "\t" << _pose.angle();
  return os;
}

