/*
 * uTest_UK_Turrets2ICRSpeed.hpp
 *
 *  Created on: 8 may 2012
 *      Author: romain
 *
 *      Teste les modèles cinématiques du robot
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"

using namespace arp_math;
using namespace arp_model;

void check_twist2turrets2ICRSpeed2twist(double vx, double vy, double vh)
{
    arp_model::UbiquityParams params;
    arp_math::Twist2D inTwist;
    arp_math::Twist2D outTwist;
    arp_model::TurretState turretCmd;
    ICRSpeed twix;

    bool res;

    inTwist.vx(vx);
    inTwist.vy(vy);
    inTwist.vh(vh);

    res = arp_model::UbiquityKinematics::twist2Turrets(inTwist, turretCmd, params);

    res = arp_model::UbiquityKinematics::simpleTurrets2ICRspeed(turretCmd, twix, params);
    outTwist = twix.twist();

    /*
     cout << "inTwist"<<inTwist.toString()<<endl;
     cout << "inTwist->twix"<<ICRSpeed(inTwist).toString()<<endl;
     cout << "turrets->twix"<<twix.toString()<<endl;
     cout << "outTwist"<<outTwist.toString()<<endl;
     */

    BOOST_CHECK_EQUAL(res, true);
    BOOST_CHECK_SMALL(outTwist.vx() - inTwist.vx(), 1.e-10);
    BOOST_CHECK_SMALL(outTwist.vy() - inTwist.vy(), 1.e-10);
    BOOST_CHECK_SMALL(outTwist.vh() - inTwist.vh(), 1.e-10);
}

void check_noMotion(double vx, double vy, double vh)
{
    arp_model::UbiquityParams params;

    arp_math::Twist2D inTwist;
    arp_math::Twist2D inTwist_inv;
    arp_model::TurretState turretCmd;
    ICRSpeed twixDesire;
    ICRSpeed twixDesire_inv;
    ICRSpeed twixObtenu;

    bool res;

    inTwist.vx(vx);
    inTwist.vy(vy);
    inTwist.vh(vh);

    //je calcule le twix que je veux au final
    twixDesire = ICRSpeed(inTwist);
    twixDesire.ro(0);
    //l'ICR oppose est aussi une solution
    inTwist_inv = Twist2D(-inTwist.vx(), -inTwist.vy(), -inTwist.vh());
    twixDesire_inv = ICRSpeed(inTwist_inv);
    twixDesire_inv.ro(0);

    //je calcule les tourelles
    arp_model::UbiquityKinematics::twist2Turrets(inTwist, turretCmd, params);
    //je fous les tourelles a 0
    turretCmd.driving.left.velocity = 0;
    turretCmd.driving.right.velocity = 0;
    turretCmd.driving.rear.velocity = 0;

    //et la je calcule le twix
    arp_model::UbiquityKinematics::simpleTurrets2ICRspeed(turretCmd, twixObtenu, params);
    /*
     cout << "-----------TEST----------" << endl;
     cout << "inTwist" << inTwist.toString() << endl;
     cout << "inTwist_inv" << inTwist_inv.toString() << endl;
     cout << "twixDesire" << twixDesire.toString() << endl;
     cout << "twixDesire_inv" << twixDesire_inv.toString() << endl;
     cout << "turretCmd" << turretCmd.toString() << endl;
     cout << "twixObtenu" << twixObtenu.toString() << endl;
     */
    BOOST_CHECK_SMALL(twixObtenu.ro() - 0, 1.e-10);
    BOOST_CHECK_SMALL(twixDesire.ro() - 0, 1.e-10);
    BOOST_CHECK_SMALL(twixDesire_inv.ro() - 0, 1.e-10);

    //le petit teste pour voir si je compare au normal ou a l'inverse ... un peu complique je sais..
    bool deltaLikeNormal = abs(twixObtenu.delta() - twixDesire.delta()) < 1.e-10;
    bool phiLikeNormal = abs(twixObtenu.phi() - twixDesire.phi()) < 1.e-10;
    bool carePhi = twixDesire.delta() < PI / 2 - 1.e-5 and twixDesire.delta() > -PI / 2 + 1.e-5;
    /*
     cout << "---"<< endl;
     cout <<"deltaLikeNormal"<<deltaLikeNormal<<endl;
     cout <<"phiLikeNormal"<<phiLikeNormal<<endl;
     cout <<"carePhi"<<carePhi<<endl;
     cout << "---"<< endl;
     */
    if (deltaLikeNormal and (phiLikeNormal or !carePhi))
    {
        // soit c'est le twix normal
        //cout << "je checke  twix normal" << endl;
        if (twixDesire.delta() < PI / 2 - 1.e-5 and twixDesire.delta() > -PI / 2 + 1.e-5) //aux poles on se tape de phi
        {
            //cout << "je checke  phi" << endl;
            BOOST_CHECK_SMALL(betweenMinusPiAndPlusPi(twixObtenu.phi() - twixDesire.phi()), 1.e-10);
        }
        else
        {
            //cout << "je checke pas phi" << endl;

        }
        BOOST_CHECK_SMALL(twixObtenu.delta() - twixDesire.delta(), 1.e-10);
    }
    else
    {
        //cout << "je checke  twix inverse" << endl;
        //soit le twix antipode
        if (twixDesire_inv.delta() < PI / 2 - 1.e-5 and twixDesire_inv.delta() > -PI / 2 + 1.e-5) //aux poles on se tape de phi
        {
            //cout << "je checke  phi" << endl;
            BOOST_CHECK_SMALL(betweenMinusPiAndPlusPi(twixObtenu.phi() - twixDesire_inv.phi()), 1.e-10);
        }
        else
        {
            //cout << "je checke pas phi" << endl;

        }
        BOOST_CHECK_SMALL(twixObtenu.delta() - twixDesire_inv.delta(), 1.e-10);
    }
}

BOOST_AUTO_TEST_CASE( UK_Turrets2ICRSpeed_Motions )
{
    check_twist2turrets2ICRSpeed2twist(0.0,0.0,-2.0);
    check_twist2turrets2ICRSpeed2twist(0.0,0.0,10.0);
    check_twist2turrets2ICRSpeed2twist(3.0,0.0,0.0);
    check_twist2turrets2ICRSpeed2twist(-1.8,0.0,0.0);
    check_twist2turrets2ICRSpeed2twist(0.0,1.2,0.0);
    check_twist2turrets2ICRSpeed2twist(0.0,-15.8,0.0);
    check_twist2turrets2ICRSpeed2twist(0.0,-15.8,0.0);
    check_twist2turrets2ICRSpeed2twist(-4.212,0.7897,1.5456);
    check_twist2turrets2ICRSpeed2twist(-8.541,1.212,-9.121);
    check_twist2turrets2ICRSpeed2twist(-0.2654,5.21,0.0123);
}

BOOST_AUTO_TEST_CASE( UK_Turrets2ICRSpeed_NoMotion)
{
    check_noMotion(0.0,0.0,-2.0);
    check_noMotion(0.0,0.0,10.0);
    check_noMotion(3.0,0.0,0.0);
    check_noMotion(-1.8,0.0,0.0);
    check_noMotion(0.0,1.2,0.0);
    check_noMotion(0.0,-15.8,0.0);
    check_noMotion(0.0,-15.8,0.0);
    check_noMotion(-4.212,0.7897,1.5456);
    check_noMotion(-8.541,1.212,-9.121);
    check_noMotion(-0.2654,5.21,0.0123);

}

