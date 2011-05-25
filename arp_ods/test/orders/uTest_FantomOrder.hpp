/*
 * uTest_FantomOrder.hpp
 *
 *  Created on: 24 may 2011
 *      Author: wla
 */

#include "orders/orders.h"
#include "math/math.hpp"
using namespace arp_ods;
using namespace arp_math;

BOOST_AUTO_TEST_CASE( FantomOrderConstructors )
{
    // Test default constructor
    FantomOrder a;
    BOOST_CHECK( a.getMode() == MODE_INIT );
    BOOST_CHECK( a.getType() == FANTOM );
    BOOST_CHECK( a.getTypeString() == "FANTOM" );
}

BOOST_AUTO_TEST_CASE( FantomOrderComputeSpeed )
{
    // Test default constructor
    MotionOrder o;
    Pose currentPosition;
    currentPosition.x = 1.3;
    currentPosition.y = -0.7;
    currentPosition.theta = 0.2;
    Velocity v = o.computeSpeed(currentPosition);
    BOOST_CHECK_EQUAL( v.linear, 0.0 );
    BOOST_CHECK_EQUAL( v.angular, 0.0 );
}

BOOST_AUTO_TEST_CASE( FantomOrderLinearReductionCoef )
{
    double error = 0.0;
    BOOST_CHECK_CLOSE( FantomOrder::linearReductionCoef(error) , 1.0 , 0.0001f);
    error = PI/16;
    BOOST_CHECK_CLOSE( FantomOrder::linearReductionCoef(error) , 0.5 , 0.0001f);
    error = - PI/16;
    BOOST_CHECK_CLOSE( FantomOrder::linearReductionCoef(error) , 0.5 , 0.0001f);
    error = PI/8;
    BOOST_CHECK_CLOSE( FantomOrder::linearReductionCoef(error) , 0.0 , 0.0001f);
    error = - PI/8;
    BOOST_CHECK_CLOSE( FantomOrder::linearReductionCoef(error) , 0.0 , 0.0001f);
    error = PI/2;
    BOOST_CHECK_CLOSE( FantomOrder::linearReductionCoef(error) , 0.0 , 0.0001f);
    error = -PI/2;
    BOOST_CHECK_CLOSE( FantomOrder::linearReductionCoef(error) , 0.0 , 0.0001f);
}

BOOST_AUTO_TEST_CASE( FantomOrderCheckApproachMode )
{
    FantomOrder fo;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.500;
    endPose.theta = 0.0;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.3;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(false);

    BOOST_CHECK_EQUAL( fo.getMode() , MODE_INIT );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) , 0.0001f);

    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) , 0.0001f);

    currentPose.x = 1.000;
    currentPose.y = 0.520;
    currentPose.theta = 0.3;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , 0.0 , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , -0.3 , 0.0001f);
    currentPose.x = 1.000;
    currentPose.y = -10.0;
    currentPose.theta = -0.3;
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , 0.0 , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0.3 , 0.0001f);
    currentPose.x = 0.900;
    currentPose.y = 0.500;
    currentPose.theta = 10*PI+0.1;
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , 0.100 , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , -0.1 , 0.0001f);
    currentPose.x = 1.100;
    currentPose.y = 0.500;
    currentPose.theta = 0;
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , -0.100 , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0.0 , 0.0001f);
    currentPose.x = 1.100;
    currentPose.y = 0.520;
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , -0.100 , 0.0001f);
}


BOOST_AUTO_TEST_CASE( FantomOrderCheckPassMode )
{
    FantomOrder fo;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.500;
    endPose.theta = 1.2;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(true);

    BOOST_CHECK_EQUAL( fo.getMode() , MODE_INIT );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) , 0.0001f);

    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) , 0.0001f);

    currentPose.x = 1.000;
    currentPose.y = 0.520;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_PASS );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , currentPose.y - endPose.y , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 1.2 , 0.0001f);
    currentPose.theta = 1.2;
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);

    sleep(1);
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_DONE );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , currentPose.y - endPose.y , 0.0001f);
}


BOOST_AUTO_TEST_CASE( FantomOrderCheckTranslationSpeedCmd )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.000;
    endPose.theta = 0.0;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(false);
    fo.setTRANSLATION_GAIN(1.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(0.0);
    fo.setVEL_FINAL(0.0);
    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,1,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,sqrt(0.5),0.0001f);
    currentPose.x = 1.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0.0,0.0001f);
    //cas tordu ou on a raté le mode approche
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    double e_d = fo.getRemainingDistance(currentPose);
    double e_theta = fo.getRemainingAngle(currentPose, e_d);
    BOOST_CHECK_CLOSE(e_d,0.5,0.0001f);//oui oui c'est plus parce que c'est l'erreur de cap qui est sencé nous ramener à destination
    BOOST_CHECK_CLOSE(e_theta,PI,0.0001f);//si on a dépassé c'est l'erreur de cap qui fait porte le demi tour
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , PI , 0.0001f);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);//du coup le linear smooth met la vitesse à 0 le temps que le robot fasse demi tour

    //passage en mode approche
    currentPose.x = 1.000;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0.0,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(fo.getRemainingDistance(currentPose),-0.5,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    BOOST_CHECK_CLOSE(v.linear,-sqrt(0.5),0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,sqrt(0.5),0.0001f);


    //cas gain nul
    fo.setTRANSLATION_GAIN(0.0);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
}

BOOST_AUTO_TEST_CASE( FantomOrderCheckNullFantomSpeedCmd )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.000;
    endPose.theta = PI/2;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(false);
    fo.setTRANSLATION_GAIN(0.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(1.0);
    fo.setVEL_FINAL(0.0);


    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );

    //Cas du point fantome direct
    fo.setFANTOM_COEF(0.0);
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    currentPose.x = 1.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    //cas tordu ou on a raté le mode approche
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI,0.0001f);


    //passage en mode approche
    currentPose.x = 1.000;
    currentPose.theta = 0;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI/2,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , PI/2 , 0.0001f);
    currentPose.theta = PI/2;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    currentPose.x = 2.300;
    currentPose.theta = PI;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,-PI/2,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , -PI/2 , 0.0001f);


    //cas gain nul
    fo.setROTATION_GAIN(0.0);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
}


BOOST_AUTO_TEST_CASE( FantomOrderCheckFullFantomSpeedCmd )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 1.000;
    endPose.theta = PI/2;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(false);
    fo.setTRANSLATION_GAIN(0.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(1.0);
    fo.setVEL_FINAL(0.0);


    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );

    fo.setFANTOM_COEF(1/sqrt(2));
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(2) , 0.0001f);
    BOOST_CHECK( fabs(fo.getRemainingAngle(currentPose,sqrt(2)) ) < 0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK( fabs(v.angular) < 0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,atan2(1-fo.getFANTOM_COEF()*fo.getRemainingDistance(currentPose),0.5),0.001f);
    currentPose.x = 1.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI/2,0.001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI - atan2(1-fo.getFANTOM_COEF()*fo.getRemainingDistance(currentPose),0.5),0.001f);
    //cas tordu ou on a raté le mode approche
    currentPose.x = 1.000;
    currentPose.y = 1.500;
    currentPose.theta = PI/2;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(normalizeAngle(v.angular + PI), 0 ,0.001f);

    //passage en mode approche
    currentPose.x = 1.000;
    currentPose.y = 0.955;
    currentPose.theta = 0;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI/2,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , PI/2 , 0.0001f);
    currentPose.theta = PI/2;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    currentPose.y = 1.500;
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);


    //cas gain nul
    fo.setROTATION_GAIN(0.0);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
}


BOOST_AUTO_TEST_CASE( FantomOrderCheckReverseTranslationSpeedCmd )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 1.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 0.000;
    endPose.y = 0.000;
    endPose.theta = 0.0;
    Pose currentPose;
    currentPose.x = 1.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(false);
    fo.setReverse(true);
    fo.setTRANSLATION_GAIN(1.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(0.0);
    fo.setVEL_FINAL(0.0);

    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , 1.0 , 0.0001f);
    BOOST_CHECK( fabs(fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) )) <=  0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-1,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-sqrt(0.5),0.0001f);
    currentPose.x = 0.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0.0,0.0001f);
    //cas tordu ou on a raté le mode approche
    currentPose.x = -0.500;
    v = fo.computeSpeed(currentPose);
    double e_d = fo.getRemainingDistance(currentPose);
    double e_theta = fo.getRemainingAngle(currentPose, e_d);
    BOOST_CHECK_CLOSE(e_d,0.5,0.0001f);//oui oui c'est plus parce que c'est l'erreur de cap qui est sencé nous ramener à destination
    BOOST_CHECK_CLOSE(normalizeAngle(e_theta+PI),0,0.0001f);//si on a dépassé c'est l'erreur de cap qui fait porte le demi tour
    BOOST_CHECK_CLOSE(normalizeAngle(fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) )+ PI), 0 , 0.0001f);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);//du coup le linear smooth met la vitesse à 0 le temps que le robot fasse demi tour

    //passage en mode approche
    currentPose.x = 0.000;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0.0,0.0001f);
    currentPose.x = -0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(fo.getRemainingDistance(currentPose),-0.5,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    BOOST_CHECK_CLOSE(v.linear,sqrt(0.5),0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-sqrt(0.5),0.0001f);


    //cas gain nul
    fo.setTRANSLATION_GAIN(0.0);
    currentPose.x = -0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
}


BOOST_AUTO_TEST_CASE( FantomOrderCheckReverseNullFantomSpeedCmd )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 1.000;
    beginPose.y = 0.000;
    beginPose.theta = 0.0;
    Pose endPose;
    endPose.x = 0.000;
    endPose.y = 0.000;
    endPose.theta = PI/2;
    Pose currentPose;
    currentPose.x = 1.000;
    currentPose.y = 0.000;
    currentPose.theta = 0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(false);
    fo.setReverse(true);
    fo.setTRANSLATION_GAIN(0.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(1.0);
    fo.setVEL_FINAL(0.0);


    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );

    //Cas du point fantome direct
    fo.setFANTOM_COEF(0.0);
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , 1 , 0.0001f);
    BOOST_CHECK( fabs(fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) )) <= 0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK(fabs(v.angular) < 0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK(fabs(v.angular) < 0.0001f);
    currentPose.x = 0.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK(fabs(v.angular) < 0.0001f);
    //cas tordu ou on a raté le mode approche
    currentPose.x = -0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(normalizeAngle(v.angular)+PI,0,0.0001f);


    //passage en mode approche
    currentPose.x = 0.000;
    currentPose.theta = 0;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI/2,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , PI/2 , 0.0001f);
    currentPose.theta = PI/2;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    currentPose.x = 2.300;
    currentPose.theta = 0;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI/2,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , PI/2 , 0.0001f);


    //cas gain nul
    fo.setROTATION_GAIN(0.0);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
}


BOOST_AUTO_TEST_CASE( FantomOrderCheckReverseFullFantomSpeedCmd )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 1.000;
    beginPose.y = 1.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 0.000;
    endPose.y = 0.000;
    endPose.theta = PI/2;
    Pose currentPose;
    currentPose.x = 1.000;
    currentPose.y = 1.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(false);
    fo.setReverse(true);
    fo.setTRANSLATION_GAIN(0.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(1.0);
    fo.setVEL_FINAL(0.0);


    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );

    fo.setFANTOM_COEF(1/sqrt(2));
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(2) , 0.0001f);
    BOOST_CHECK( fabs(fo.getRemainingAngle(currentPose,sqrt(2)) ) < 0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK( fabs(v.angular) < 0.0001f);
    currentPose.theta = 1;
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(2) , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,sqrt(2)) , -1 ,  0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular, -1, 0.0001f);
    currentPose.theta = -0.3;
    BOOST_CHECK_CLOSE( fo.getRemainingDistance(currentPose) , sqrt(2) , 0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,sqrt(2)) , 0.3 ,  0.0001f);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular, 0.3, 0.0001f);


    currentPose.x = 0.500;
    currentPose.theta = 0;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,atan2(1-fo.getFANTOM_COEF()*fo.getRemainingDistance(currentPose),0.5),0.001f);
    currentPose.x = 0.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI/2,0.001f);
    currentPose.x = -0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI - atan2(1-fo.getFANTOM_COEF()*fo.getRemainingDistance(currentPose),0.5),0.001f);
    //cas tordu ou on a raté le mode approche
    currentPose.x = 0.000;
    currentPose.y = -0.500;
    currentPose.theta = PI/2;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(normalizeAngle(v.angular + PI), 0 ,0.001f);

    //passage en mode approche
    currentPose.x = 0.000;
    currentPose.y = 0.045;
    currentPose.theta = 0;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,PI/2,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , PI/2 , 0.0001f);
    currentPose.theta = PI/2;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);
    currentPose.y = -0.500;
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
    BOOST_CHECK_CLOSE( fo.getRemainingAngle(currentPose,fo.getRemainingDistance(currentPose) ) , 0 , 0.0001f);


    //cas gain nul
    fo.setROTATION_GAIN(0.0);
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.angular,0,0.0001f);
}


BOOST_AUTO_TEST_CASE( FantomOrderCheckVFinal )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.000;
    endPose.theta = 0.0;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    //la vitesse minimum n'est active que dans le cas pass
    fo.setPass(true);
    fo.setTRANSLATION_GAIN(0.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(0.0);
    double VEL_FINAL = 0.10;
    fo.setVEL_FINAL(VEL_FINAL);

    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);

    //passage en mode approche
    currentPose.x = 1.000;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_PASS );

    currentPose.x = 0.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,VEL_FINAL,0.0001f);

    //dans le cas normal on a toujours 0
    fo.setPass(false);
    fo.resetMode();
    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);

    //passage en mode approche
    currentPose.x = 1.000;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );

    currentPose.x = 0.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);

}

BOOST_AUTO_TEST_CASE( FantomOrderCheckVFinalReverse )
{
    FantomOrder fo;
    Velocity v;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.000;
    endPose.theta = 0.0;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.0;


    fo.setBeginPose(beginPose);
    fo.setEndPose(endPose);
    fo.setRadiusInitZone(0.010);
    fo.setRadiusApproachZone(0.050);
    fo.setAngleAccuracy(0.2);
    fo.setDistanceAccurancy(0.010);
    fo.setPass(true);
    fo.setReverse(true);
    fo.setTRANSLATION_GAIN(0.0);
    fo.setROTATION_D_GAIN(0.0);
    fo.setROTATION_GAIN(0.0);
    double VEL_FINAL = 0.10;
    fo.setVEL_FINAL(0.10);


    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);

    //passage en mode approche
    currentPose.x = 1.000;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_PASS );

    currentPose.x = 0.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,-VEL_FINAL,0.0001f);

    //dans le cas normal on a toujours 0
    fo.setPass(false);
    fo.resetMode();
    //fo switch automatiquement
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_RUN );
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);

    //passage en mode approche
    currentPose.x = 1.000;
    fo.switchMode(currentPose);
    BOOST_CHECK_EQUAL( fo.getMode() , MODE_APPROACH );

    currentPose.x = 0.000;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 0.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 - 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.000 + 1E9;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
    currentPose.x = 1.500;
    v = fo.computeSpeed(currentPose);
    BOOST_CHECK_CLOSE(v.linear,0,0.0001f);
}

