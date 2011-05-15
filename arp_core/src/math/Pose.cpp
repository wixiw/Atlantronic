/*
 * Pose.cpp
 *
 *  Created on: 10 sept. 2010
 *      Author: boris
 */

#include "Pose.hpp"

namespace arp_math
{

    Pose::Pose(Pose & _pose) :
        positionTranslation(Vector2(0.0, 0.0)),
                positionRotation(Rotation2(0.0))
    {
        positionTranslation = _pose.translation();
        positionRotation = _pose.rotation2();
    }

    Pose::Pose(Vector2 _positionTranslation, Rotation2 _positionRotation) :
        positionTranslation(Vector2(0.0, 0.0)),
                positionRotation(Rotation2(0.0))
    {
        positionTranslation = _positionTranslation;
        positionRotation = _positionRotation;
    }

    Pose::~Pose()
    {
    }

    Vector2 Pose::translation()
    {
        return positionTranslation;
    }

    Rotation2 Pose::rotation2()
    {
        return positionRotation;
    }

    double Pose::x()
    {
        return positionTranslation(0);
    }

    double Pose::y()
    {
        return positionTranslation(1);
    }

    double Pose::angle()
    {
        return positionRotation.angle();
    }

    /*Displacement Pose::displacement()
     {
     return Displacement(this->getMatrix4());
     }*/

    /* TODO BMO eigen 3 compilation problem */
    //Eigen::Matrix<double,3,3> Pose::matrix3()
    //{
    //	Eigen::Matrix<double,3,3> mat = Eigen::Matrix<double,3,3>::Identity();
    //	mat.corner(Eigen::TopLeft,2,2) = this->rotation2().toRotationMatrix();
    //	mat.corner(Eigen::TopRight,2,1) = this->translation();
    //	return mat;
    //}
    //Eigen::Matrix<double,4,4> Pose::matrix4()
    //{
    //	Eigen::Matrix<double,4,4> mat = Eigen::Matrix<double,4,4>::Identity();
    //	mat.corner(Eigen::TopLeft,2,2) = this->rotation2().toRotationMatrix();
    //	mat.corner(Eigen::TopRight,2,1) = this->translation();
    //	return mat;
    //}

    void Pose::translation(Vector2 _positionTranslation)
    {
        positionTranslation = _positionTranslation;
    }

    void Pose::rotation2(Rotation2 _positionRotation)
    {
        positionRotation = _positionRotation;
    }

    void Pose::x(double _x)
    {
        positionTranslation(0) = _x;
    }

    void Pose::y(double _y)
    {
        positionTranslation(1) = _y;
    }

    void Pose::angle(double _heading)
    {
        positionRotation = Rotation2(_heading);
    }

    Pose Pose::inverse()
    {
        Pose p;
        p.translation(-this->translation());
        p.rotation2(this->rotation2().inverse());
        return p;
    }

    Pose Pose::operator =(Pose other)
    {
        this->translation(other.translation());
        this->rotation2(other.rotation2());
        return *this;
    }

    bool Pose::operator ==(Pose other)
    {
        return (this->angle() == other.angle()) && (this->translation()
                == other.translation());
    }

//inline friend Pose operator+(const Pose& lhs, const Pose& rhs)
//{
//	return Pose( lhs.translation() + rhs.translation() , lhs.angle() + rhs.angle() );
//}
//
//Pose Pose::operator -(Pose other)
//{
//	return Pose( this->translation() + other.translation() , this->angle() + other.angle() );
//}
//
//inline std::ostream& operator <<(std::ostream& os, Pose _pose)
//{
//  os << _pose.translation().transpose() << "\t" << _pose.angle();
//  return os;
//}

}
