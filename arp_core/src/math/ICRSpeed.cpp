/*
 * ICRSpeed.cpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#include <math/ICRSpeed.hpp>

using namespace arp_math;

std::ostream &operator<<( std::ostream &flux, arp_math::ICRSpeed const& t)
{
    return flux << t.toString();
}

ICRSpeed::ICRSpeed(double _ro, double _alpha,double _q):
        ro(_ro),
        alpha(_alpha),
        q(_q){}


ICRSpeed::ICRSpeed(Twist2D _twist)
        {
        double _v;
        double _ro;
        double _alpha;
        double _q;
        _v=sqrt(_twist.vx()*_twist.vx()+_twist.vy()*_twist.vy());

        if (_twist.vh()!=0.0)
            _ro=atan(_v/_twist.vh());
        else
            _ro=PI/2.0;
        if (_twist.vx()!=0 or _twist.vy()!=0)
            _alpha=atan2(_twist.vy(),_twist.vx());
        else
            _alpha=PI/2.0;
        _q=_v+_twist.vh();

        m_ro=_ro;
        m_alpha=_alpha;
        m_q=_q;

        }



double ICRSpeed::ro() const
{
    return m_ro;
}

double ICRSpeed::alpha() const
{
    return m_alpha;
}

double ICRSpeed::q() const
{
    return m_q;
}

double& ICRSpeed::roRef()
{
    return m_ro;
}

double& ICRSpeed::alphaRef()
{
    return m_alpha;
}

double& ICRSpeed::qRef()
{
    return m_q;
}

void ICRSpeed::ro(double _ro)
{
    m_ro = _ro;
}

void ICRSpeed::alpha(double _alpha)
{
    m_alpha = _alpha;
}

void ICRSpeed::q(double _q)
{
    m_q = _q;
}

Twist2D ICRSpeed::twist()
{
    double _v;
    double _vx;
    double _vy;
    double _vh;


    if (m_ro==PI/2.0)
       _v=m_q;
    else if (m_ro==0.0)
       _v=0.0;
    else
       _v=m_q/(1+1/tan(m_ro));

    _vx=_v*cos(m_alpha);
    _vy=_v*sin(m_alpha);
    _vh=m_q-_v;

    return Twist2D(_vx,_vy,_vh);
}

std::string ICRSpeed::toString() const
{
    std::ostringstream  s;
    s << "(" << ro() << "," << alpha() << "," << q() << ")";
    return s.str();
}
