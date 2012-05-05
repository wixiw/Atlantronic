/*
 * MathFactory.hpp
 *
 *  Created on: 2 Mai 2012
 *      Author: Boris
 */

#include <math/MathFactory.hpp>
using namespace arp_math;

Pose2D MathFactory::createPose2D(const Vector2 & translation, const Rotation2 & orientation)
{
    Pose2D p;
    p.x(translation[0]);
    p.y(translation[1]);
    p.orientation( orientation );
    return p;
}

EstimatedPose2D MathFactory::createEstimatedPose2D(const Vector2 & translation, const Rotation2 & orientation, const long double date, const Covariance3 & cov )
{
    Pose2D p = MathFactory::createPose2D(translation, orientation);
    EstimatedPose2D out(p);
    out.date( date );
    out.cov( cov );
    return out;
}

EstimatedPose2D MathFactory::createEstimatedPose2D(const double x, const double y, const double h, const long double date, const Covariance3 & cov )
{
    Pose2D p(x, y, h);
    EstimatedPose2D out(p);
    out.date( date );
    out.cov( cov );
    return out;
}


Twist2D MathFactory::createTwist2DFromCartesianRepr(const double vx, const double vy, const double vh)
{
    Twist2D t;
    t.vx(vx);
    t.vy(vy);
    t.vh(vh);
    return t;
}

Twist2D MathFactory::createTwist2DFromCartesianRepr(const Vector2 & _vitesseTranslation, const double _vitesseRotation)
{
    Twist2D t;
    t.vx(_vitesseTranslation[0]);
    t.vy(_vitesseTranslation[1]);
    t.vh(_vitesseRotation);
    return t;
}

Twist2D MathFactory::createTwist2DFromCartesianRepr(const Vector3 & T)
{
    Twist2D t;
    t.vx(T[0]);
    t.vy(T[1]);
    t.vh(T[2]);
    return t;
}

Twist2D MathFactory::createTwist2DFromPolarRepr(const double normV, const double angV, const double vh)
{
    Twist2D t;
    t.vx(normV*cos(angV));
    t.vy(normV*sin(angV));
    t.vh(vh);
    return t;
}

EstimatedTwist2D MathFactory::createEstimatedTwist2DFromCartesianRepr(const double vx, const double vy, const double vh, const long double date, const Covariance3 & cov)
{
    Twist2D t = MathFactory::createTwist2DFromCartesianRepr(vx, vy, vh);
    EstimatedTwist2D out = EstimatedTwist2D(t);
    out.date( date );
    out.cov( cov );
    return out;
}

EstimatedTwist2D MathFactory::createEstimatedTwist2DFromCartesianRepr(const Vector2 & vitesseTranslation, const double vitesseRotation, const long double date, const Covariance3 & cov)
{
    Twist2D t = MathFactory::createTwist2DFromCartesianRepr(vitesseTranslation, vitesseRotation);
    EstimatedTwist2D out = EstimatedTwist2D(t);
    out.date( date );
    out.cov( cov );
    return out;
}

EstimatedTwist2D MathFactory::createEstimatedTwist2DFromCartesianRepr(const Vector3 & T, const long double date, const Covariance3 & cov)
{
    Twist2D t = MathFactory::createTwist2DFromCartesianRepr(T);
    EstimatedTwist2D out = EstimatedTwist2D(t);
    out.date( date );
    out.cov( cov );
    return out;
}


EstimatedTwist2D MathFactory::createEstimatedTwist2DFromPolarRepr(const double normV, const double angV, const double vh, const long double date, const Covariance3 & cov)
{
    Twist2D t = MathFactory::createTwist2DFromPolarRepr(normV, angV, vh);
    EstimatedTwist2D out = EstimatedTwist2D(t);
    out.date( date );
    out.cov( cov );
    return out;
}
