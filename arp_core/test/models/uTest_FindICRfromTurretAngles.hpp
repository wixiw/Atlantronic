
#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"

using namespace arp_core;
using namespace arp_math;
using namespace arp_model;

BOOST_AUTO_TEST_CASE( UK_findICR_DegeneratedCases)
{
    UbiquityParams params;
    params.fillWithFakeValues();
    TurretState iTS;
    ICR icrPos;

    //Cas particulier des tourelles parralèles
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_SMALL(icrPos.phi(), 1E-5);
    //TODO cas particulier à traiter
    BOOST_CHECK_CLOSE(icrPos.delta(), 123456, 1E-5);


    //Cas particulier des tourelles alignées
    iTS.steering.left.position = +M_PI_2;
    iTS.steering.right.position = -M_PI_2;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), M_PI_2, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);

    //Cas particulier des tourelles alignées
    iTS.steering.left.position = -M_PI_2;
    iTS.steering.right.position = +M_PI_2;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), -M_PI_2, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);

    //Cas particulier des tourelles alignées
    iTS.steering.left.position = +M_PI_2;
    iTS.steering.right.position = +M_PI_2;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), M_PI_2, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);

    //Cas particulier des tourelles alignées
    iTS.steering.left.position = -M_PI_2;
    iTS.steering.right.position = -M_PI_2;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), -M_PI_2, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);

    //Sur la tourelle de gauche
    iTS.steering.left.position = 0.;
    iTS.steering.right.position = M_PI_2;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_SMALL(icrPos.phi(), 1E-5);
    BOOST_CHECK_CLOSE(icrPos.delta(), -atan(Twist2DNorm::dmax/params.getLeftTurretPosition().y()), 0.5);
}

BOOST_AUTO_TEST_CASE( UK_findICR_TranslationTests)
{
    UbiquityParams params;
    params.fillWithFakeValues();
    TurretState iTS;
    ICR icrPos;
    double phi;

    phi = M_PI_2;
    iTS.steering.left.position = phi;
    iTS.steering.right.position = phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), phi, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);

    phi = -M_PI_2;
    iTS.steering.left.position = phi;
    iTS.steering.right.position = phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), phi, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);

    phi = -3*M_PI_4;
    iTS.steering.left.position = phi;
    iTS.steering.right.position = phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), phi, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);

    phi = -3*M_PI_4;
    iTS.steering.left.position = phi;
    iTS.steering.right.position = phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), phi, 1E-5);
    BOOST_CHECK_SMALL(icrPos.delta(), 1E-5);
}

//turret are symetrical +angle/-angle
BOOST_AUTO_TEST_CASE( UK_findICR_VTets)
{
    UbiquityParams params;
    params.fillWithFakeValues();
    TurretState iTS;
    ICR icrPos;
    double phi;

    phi = 1E-2;
    iTS.steering.left.position = +phi;
    iTS.steering.right.position = -phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    //phi can be whatever it wants.
    BOOST_CHECK_CLOSE(icrPos.phi(), M_PI_2, 0.5);
    BOOST_CHECK_CLOSE(icrPos.delta(), -M_PI_2, 0.5);

    phi = 1E-2;
    iTS.steering.left.position = -phi;
    iTS.steering.right.position = +phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    //phi can be whatever it wants.
    BOOST_CHECK_CLOSE(icrPos.phi(), -M_PI_2, 0.5);
    BOOST_CHECK_CLOSE(icrPos.delta(), -M_PI_2, 0.5);


    phi = M_PI_4;
    iTS.steering.left.position = +phi;
    iTS.steering.right.position = -phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), M_PI_2, 0.5);
    BOOST_CHECK_CLOSE(icrPos.delta(), -atan(Twist2DNorm::dmax/params.getLeftTurretPosition().y()), 1E-5);

    phi = -M_PI_4;
    iTS.steering.left.position = +phi;
    iTS.steering.right.position = -phi;
    UbiquityKinematics::findICRfromTurretAngles(iTS, icrPos,
            iTS.steering.left.position, params.getLeftTurretPosition(),
            iTS.steering.right.position, params.getRightTurretPosition());
    BOOST_CHECK_CLOSE(icrPos.phi(), -M_PI_2, 0.5);
    BOOST_CHECK_CLOSE(icrPos.delta(), -atan(Twist2DNorm::dmax/params.getLeftTurretPosition().y()), 1E-5);
}
