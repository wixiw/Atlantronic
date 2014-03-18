/*
 * ICR.cpp
 *
 *  Created on: 7 september 2012
 *      Author: romain
 */

#include "math/ICR.hpp"
#include "math/math.hpp"
#include <iostream>

using namespace arp_math;
using namespace Eigen;
using namespace std;

std::ostream &operator<<(std::ostream &flux, arp_math::ICR const& t)
{
    return flux << t.toString();
}

ICR::ICR(double phi, double delta)
{
    m_phi = betweenMinusPiAndPlusPi(phi);
    m_delta = betweenMinusPiAndPlusPi(delta);
}

ICR::ICR(const ICR& icr)
{
    m_phi = icr.phi();
    m_delta = icr.delta();
}

ICR::ICR(Vector3 speedVector)
{
    //TODO double equal
    //phi
    if ( d_abs(speedVector[0]) > epsilon or d_abs(speedVector[1]) > 0.0)
        m_phi = atan2(speedVector[1], speedVector[0]);
    else
        // pure rotation, phi not defined, but not a problem.
        m_phi = 0.0;

    //delta
    double v = sqrt(speedVector[0] * speedVector[0] + speedVector[1] * speedVector[1]);
    if (d_abs(v) > epsilon)
        m_delta = atan(speedVector[2] / v);
    else
        m_delta = sign(speedVector[2]) * PI / 2.0;

}

ICR ICR::getAntipodICR() const
{
    double phi = betweenMinusPiAndPlusPi(m_phi + PI);
    double delta = -m_delta;

    return ICR(phi, delta);
}
Vector3 ICR::getCartesianVector() const
{
    double dx;
    double dy;
    double dh;

    dx = cos(delta()) * cos(phi());
    dy = cos(delta()) * sin(phi());
    dh = sin(delta());

    return Vector3(dx, dy, dh);
}

double ICR::sphericalDistance(const ICR& ICR2) const
{
    double dotprod = sin(delta()) * sin(ICR2.delta()) + cos(delta()) * cos(ICR2.delta()) * cos(phi() - ICR2.phi());
    return acos(dotprod);
}

ICR ICR::getIntermediate(const ICR& ICR2, double s) const
{
    Vector3 M1 = getCartesianVector();
    Vector3 M2 = ICR2.getCartesianVector();

    if (M1 == M2)
        return M1;

    Vector3 M1cM2 = M1.cross(M2);

    // points where opposite
    if (M1cM2 == Vector3(0, 0, 0))
        M1cM2 = M1.cross(Vector3(0, 0, 1));
    //points where both poles
    if (M1cM2 == Vector3(0, 0, 0))
        M1cM2 = M1.cross(Vector3(1, 0, 0));

    // N is the unitary vector that is center of rotation to tranform M1 into M2
    Vector3 N = 1 / M1cM2.norm() * M1cM2;

    // s should be saturated so as not to go beyond M2
    s = min (s,angleBetweenVectors(M1,M2));

    //apply rotation of M1, of a distance s
    Vector3 M3 = cos(s) * M1 + (1 - cos(s)) * (M1.dot(N)) * N + sin(s) * N.cross(M1);

    //TODO handle s>PI, take shortest path
    return ICR(M3);
}



double ICR::phi() const
{
    return m_phi;
}

double ICR::delta() const
{
    return m_delta;
}

double& ICR::phiRef()
{
    return m_phi;
}

double& ICR::deltaRef()
{
    return m_delta;
}

void ICR::phi(double phi)
{
    m_phi = betweenMinusPiAndPlusPi(phi);
}

void ICR::delta(double delta)
{
    m_delta = betweenMinusPiAndPlusPi(delta);
}

ICR ICR::transport(const Pose2D & p) const
{
    double a,b,c;
    transport(p, a, b, c);
}

ICR ICR::transport(const Pose2D & p, double& a, double &b, double& c) const
{
    ICR transportedIcr;

    a = cos(delta()) * cos(phi()) - p.y()/Twist2DNorm::dmax * sin(delta());
    b = cos(delta()) * sin(phi()) + p.x()/Twist2DNorm::dmax * sin(delta());
    c = sin(delta());

    transportedIcr.phi( atan2(b,a) - p.angle() );
    transportedIcr.delta( atan2(c,sqrt(a*a + b*b)) );
    cout << "a=" << a << " b=" << b << " c=" << c << endl;
    cout << "p= " << p.toString() << endl;
    cout << "atan2=" << atan2(b,a) << " p.angle=" << p.angle() <<endl;
    cout << "transportedIcr=" << transportedIcr.toString() <<endl;
    return transportedIcr;
}


std::string ICR::toString() const
{
    std::ostringstream s;
    s << "ICR en (phi=" << rad2deg(phi()) << "° ,delta=" << rad2deg(delta()) << "° )";
    return s.str();
}
