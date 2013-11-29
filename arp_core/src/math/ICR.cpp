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
    m_phi = phi;
    m_delta = delta;
}

ICR::ICR(Vector3 cartesianVector)
{
    //phi
    if (cartesianVector[0] != 0.0 or cartesianVector[1] != 0.0)
        m_phi = atan2(cartesianVector[1], cartesianVector[0]);
    else
        // pure rotation, phi not defined, but not a problem.
        m_phi = 0.0;

    //delta
    double v = sqrt(cartesianVector[0] * cartesianVector[0] + cartesianVector[1] * cartesianVector[1]);
    if (v != 0.0)
        m_delta = atan(cartesianVector[2] / v);
    else
        m_delta = sign(cartesianVector[2]) * PI / 2.0;

}

ICR ICR::getAntipodICR()
{
    double phi = betweenMinusPiAndPlusPi(m_phi + PI);
    double delta = -m_delta;

    return ICR(phi, delta);
}
Vector3 ICR::getCartesianVector()
{
    double dx;
    double dy;
    double dh;

    dx = cos(delta()) * cos(phi());
    dy = cos(delta()) * sin(phi());
    dh = sin(delta());

    return Vector3(dx, dy, dh);
}

double ICR::sphericalDistance(ICR ICR2)
{
    double dotprod = sin(delta()) * sin(ICR2.delta()) + cos(delta()) * cos(ICR2.delta()) * cos(phi() - ICR2.phi());
    return acos(dotprod);
}

ICR ICR::getIntermediate(ICR ICR2, double s)
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
    m_delta = delta;
}

std::string ICR::toString() const
{
    std::ostringstream s;
    s << "ICR en (phi=" << rad2deg(phi()) << "° ,delta=" << rad2deg(delta()) << "° )";
    return s.str();
}
