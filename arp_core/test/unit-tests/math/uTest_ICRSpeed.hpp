/*
 * test_ICRSpeed.hpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#include <math/core>
#include <iostream>
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( ICRSpeed_Default_Constructor )
{
    // Test default constructor
    ICRSpeed a;
    BOOST_CHECK( a.ro() == 0.0 );
    BOOST_CHECK( a.phi() == 0.0 );
    BOOST_CHECK( a.delta() == 0.0 );
}

BOOST_AUTO_TEST_CASE( ICRSpeed_Accessors )
{
    ICRSpeed a;
    a.ro(1.);
    a.phi(2.);
    a.delta(3.);
    BOOST_CHECK( a.ro() == 1. );
    BOOST_CHECK( a.phi() == 2. );
    BOOST_CHECK( a.delta() == 3. );
}

void testConsistency(double vx, double vy, double vh)
{
    Twist2DNorm twist_from;
    ICRSpeed ICR_to;
    Twist2DNorm twist_back;

    twist_from = Twist2DNorm(vx, vy, vh);
    ICR_to = ICRSpeed(twist_from);
    twist_back = ICR_to.twistNorm();
    //c'est bien gentil boost check close mais la tolerance est un pourcentage des arguments et comme j'ai un argument à 0 j'ai un fail alors que ca allait bien
    //http://www.boost.org/doc/libs/1_34_1/libs/test/doc/components/test_tools/floating_point_comparison.html

    /*
     cout << "--------TEST--------" << endl;
     cout << "vx=  " << vx << "wy=  " << vy << "vh=  " << vh << endl;
     cout << "twist_from=  " << twist_from.toString() << endl;
     cout << "ICR_to=  " << ICR_to.toString() << endl;
     cout << "twist_back=  " << twist_back.toString() << endl;
     */

    BOOST_CHECK_SMALL(twist_from.vx() - twist_back.vx(), 1e-6);
    BOOST_CHECK_SMALL(twist_from.vy() - twist_back.vy(), 1e-6);
    BOOST_CHECK_SMALL(twist_from.vh() - twist_back.vh(), 1e-6);

    /* jen profite pour checker le txist normal (la conversion est faite dans ICRSpeed*/
    Twist2D twist_from2;
    ICRSpeed ICR_to2;
    Twist2D twist_back2;

    twist_from2 = Twist2D(vx, vy, vh);
    ICR_to2 = ICRSpeed(twist_from2);
    twist_back2 = ICR_to2.twist();

/*
     cout << "--------TEST--------" << endl;
     cout << "vx=  " << vx << "wy=  " << vy << "vh=  " << vh << endl;
     cout << "twist_from2=  " << twist_from2.toString() << endl;
     cout << "Twist2DNorm(twist_from2)"<<Twist2DNorm(twist_from2).toString()<<endl;
     cout << "ICR_to2=  " << ICR_to2.toString() << endl;
     cout << "twist_back2=  " << twist_back2.toString() << endl;
*/

    BOOST_CHECK_SMALL(twist_from2.vx() - twist_back2.vx(), 1e-6);
    BOOST_CHECK_SMALL(twist_from2.vy() - twist_back2.vy(), 1e-6);
    BOOST_CHECK_SMALL(twist_from2.vh() - twist_back2.vh(), 1e-6);

}

BOOST_AUTO_TEST_CASE( ICRSpeed_Consistency )
{
    // default ICRSpeed should give zero speed twist
    ICRSpeed ICR_a;
    Twist2DNorm twist_a = ICR_a.twistNorm();

    BOOST_CHECK_CLOSE( twist_a.vx() , 0.0 ,1e-6);
    BOOST_CHECK_CLOSE( twist_a.vy() , 0.0 ,1e-6 );
    BOOST_CHECK_CLOSE( twist_a.vh() , 0.0 ,1e-6);

    //zero speed twist should give an ICRSpeed as the default
    Twist2DNorm twist_b(0, 0, 0);
    ICRSpeed ICR_b(twist_b);
    BOOST_CHECK_CLOSE( ICR_b.ro() , 0.0 ,1e-6);
    BOOST_CHECK_CLOSE( ICR_b.phi() , 0.0 ,1e-6 );
    BOOST_CHECK_CLOSE( ICR_b.delta() , 0.0 ,1e-6);

    // going from twist to ICRSpeed back to twist should be ok
    testConsistency(1,0,0);
    testConsistency(0,1,0);
    testConsistency(0,0,1);
    testConsistency(0,0,-1);
    testConsistency(1,1,0);
    testConsistency(-1,-1,-1);
    testConsistency(1.875,2.414,0.654);
    testConsistency(9.879,3.54651,-4.510);
    testConsistency(0.0654,-8.012,-0.5645);

}

void testSpeedDirection(double vx, double vy, double vh)
{
    Twist2DNorm twist_from = Twist2DNorm(vx, vy, vh);
    ICRSpeed ICR_to = ICRSpeed(twist_from);
    double speed = sqrt(
            twist_from.vx() * twist_from.vx() + twist_from.vy() * twist_from.vy() + twist_from.vh() * twist_from.vh());
    Vector3 speedDirTwist = 1 / speed * twist_from.getTVector();
    ;
    Vector3 speedDirICR = ICR_to.speedDirection();

    BOOST_CHECK_SMALL(speedDirTwist[0] - speedDirICR[0], 1e-6);
    BOOST_CHECK_SMALL(speedDirTwist[1] - speedDirICR[1], 1e-6);
    BOOST_CHECK_SMALL(speedDirTwist[2] - speedDirICR[2], 1e-6);

}

BOOST_AUTO_TEST_CASE( ICRSpeed_speedDirection )
{
    testSpeedDirection(1.0,0.0,0.0);
    testSpeedDirection(0.0,1.0,0.0);
    testSpeedDirection(0.0,0.0,1.0);
    testSpeedDirection(1.0,1.0,1.0);
    testSpeedDirection(10.0,0.0,0.0);
    testSpeedDirection(1,3,4);
    testSpeedDirection(-4.38383,-5.986,8.234);
    testSpeedDirection(34.2345,4.567,-4.5);
    testSpeedDirection(-1.34,0.0,-5.460);
    testSpeedDirection(0,-5.65,-6.560);
}

void testOppositeRep(double vx, double vy, double vh)
{
    Twist2DNorm twist_from = Twist2DNorm(vx, vy, vh);
    ICRSpeed ICR_to = ICRSpeed(twist_from);

    ICRSpeed ICR_equivalent = ICR_to.getOppositeRep();
    Twist2DNorm twist_equivalent = ICR_equivalent.twistNorm();

    /*
     cout<<"--------TEST--------"<<endl;
     cout<<"vx=  "<<vx<<"wy=  "<<vy<<"vh=  "<<vh<<endl;
     cout<<"twist_from=  "<<twist_from.toString()<<endl;
     cout<<"ICR_to=  "<<ICR_to.toString()<<endl;
     cout<<"ICR_equivalent=  "<<ICR_equivalent.toString()<<endl;
     cout<<"twist_equivalent=  "<<twist_equivalent.toString()<<endl;
     */

    //on a bien change de signe
    BOOST_CHECK_SMALL(ICR_to.ro() + ICR_equivalent.ro(), 1e-6);
    /// et c'est equivalent
    BOOST_CHECK_SMALL(twist_from.vx() - twist_equivalent.vx(), 1e-6);
    BOOST_CHECK_SMALL(twist_from.vy() - twist_equivalent.vy(), 1e-6);
    BOOST_CHECK_SMALL(twist_from.vh() - twist_equivalent.vh(), 1e-6);

}

BOOST_AUTO_TEST_CASE( ICRSpeed_oppositeRep )
{
    testOppositeRep(1.0,0.0,0.0);
    testOppositeRep(0.0,1.0,0.0);
    testOppositeRep(0.0,0.0,1.0);
    testOppositeRep(0.0,0.0,-1.0);
    testOppositeRep(1.0,1.0,1.0);
    testOppositeRep(10.0,0.0,0.0);
    testOppositeRep(1,3,4);
    testOppositeRep(-4.38383,-5.986,8.234);
    testOppositeRep(34.2345,4.567,-4.5);
    testOppositeRep(-1.34,0.0,-5.460);
    testOppositeRep(0,-5.65,-6.560);
}

void testICRDistance(double phi1, double delta1, double phi2, double delta2, double angle)
{
    ICR ICR1 = ICR(phi1, delta1);
    ICR ICR2 = ICR(phi2, delta2);
    double dist = ICR1.sphericalDistance(ICR2);

    /*
     cout << "--------TEST--------" << endl;
     cout << "phi1=  " << phi1 << "delta1=  " << delta1 << endl;
     cout << "ICR1=  " << ICR1.toString() << endl;
     cout << "phi2=  " << phi2 << "delta2=  " << delta2 << endl;
     cout << "ICR2=  " << ICR2.toString() << endl;

     cout << "angle expected=  " << angle << endl;
     cout << "dist=  " << dist << endl;
     */
    BOOST_CHECK_SMALL(angle - dist, 1e-6);
}

BOOST_AUTO_TEST_CASE( ICR_distance )
{
    testICRDistance(0,0,0,0,0);
    testICRDistance(0,0,PI,0,PI);
    testICRDistance(0,0,-PI/2,0,PI/2);
    testICRDistance(0,0,0,PI/2,PI/2);
    testICRDistance(0,0,0,-PI/2,PI/2);
    testICRDistance(0,0,PI,PI/4,3*PI/4);
    testICRDistance(PI/4,-PI/4,-3*PI/4,PI/4,PI);
}

void testICRIntermediate(double phi1, double delta1, double phi2, double delta2, double dist, double phi3,
        double delta3)
{
    ICR ICR1 = ICR(phi1, delta1);
    ICR ICR2 = ICR(phi2, delta2);
    ICR ICR3 = ICR1.getIntermediate(ICR2, dist);
/*
    cout << "--------TEST--------" << endl;
    cout << "phi1=  " << phi1 << "delta1=  " << delta1 << endl;
    cout << "ICR1=  " << ICR1.toString() << endl;
    cout << "phi2=  " << phi2 << "delta2=  " << delta2 << endl;
    cout << "ICR2=  " << ICR2.toString() << endl;
    cout << "dist:" << dist << endl;
    cout << "expected:" << endl;
    cout << "phi3=  " << phi3 << "delta3=  " << delta3 << endl;
    cout << "got:" << endl;
    cout << "ICR3=  " << ICR3.toString() << endl;
*/
    BOOST_CHECK_SMALL(ICR3.phi() - phi3, 1e-6);
    BOOST_CHECK_SMALL(ICR3.delta() - delta3, 1e-6);

}

BOOST_AUTO_TEST_CASE( ICR_intermediate)
{
    testICRIntermediate(0,0,0,0,0,0,0);
    testICRIntermediate(0,0,PI/2,0,0,0,0);
    testICRIntermediate(0,0,PI/2,0,PI/4,PI/4,0);
    testICRIntermediate(0,0,PI/2,0,3*PI/4,3*PI/4,0);
    testICRIntermediate(0,0,3*PI/4,0,PI/2,PI/2,0);
    testICRIntermediate(-PI/2,0,PI/2,0,PI/2,0,0);
    testICRIntermediate(0,0,0,-PI/2,PI/4,0,-PI/4);
    testICRIntermediate(PI/2,0,-PI/2,PI/4,PI/4,PI/2,PI/4);
    //from pole to pole
    testICRIntermediate(PI/4,-PI/2,-PI/4,PI/2,PI/2,0,0);

    //TODO did not manage to test the case where they are opposite, due to rounding errors, the condition "opposite" is never met
}
