/*
 * uTest_Math.hpp
 *
 *  Created on: 30 mai 2011
 *      Author: ard
 */

#ifndef _UTEST_MATH_HPP_
#define _UTEST_MATH_HPP_

#include "math/math.hpp"
using namespace arp_math;

BOOST_AUTO_TEST_CASE( Math_normalizeAngle )
{
    srand(time(0));
    for(unsigned i = 0; i < 100; i++)
    {
        double angle = (double) rand()/RAND_MAX * 10. - 5.;

        double res = betweenMinusPiAndPlusPi(angle);
        BOOST_CHECK_CLOSE( cos(res), cos(angle), 0.001f );
        BOOST_CHECK_CLOSE( sin(res), sin(angle), 0.001f );
        BOOST_CHECK( res >= -PI );
        BOOST_CHECK( res <= PI );
    }
}

BOOST_AUTO_TEST_CASE( Math_betweenZeroAndTwoPi )
{
    srand(time(0));
    for(unsigned i = 0; i < 100; i++)
    {
        double angle = (double) rand()/RAND_MAX * 10. - 5.;

        double res = betweenZeroAndTwoPi(angle);
        BOOST_CHECK_CLOSE( cos(res), cos(angle), 0.001f );
        BOOST_CHECK_CLOSE( sin(res), sin(angle), 0.001f );
        BOOST_CHECK( res >= 0. );
        BOOST_CHECK( res <= 2. * PI );
    }
}

BOOST_AUTO_TEST_CASE( Math_betweenMinusPiAndPlusPi )
{
    BOOST_CHECK_EQUAL( betweenMinusPiAndPlusPi(0.), 0. );
    BOOST_CHECK_EQUAL( betweenMinusPiAndPlusPi(-PI), PI );
    BOOST_CHECK_EQUAL( betweenMinusPiAndPlusPi( PI), PI );

    srand(time(0));
    for(unsigned i = 0; i < 100; i++)
    {
        double angle = (double) rand()/RAND_MAX * 10. - 5.;

        double res = betweenMinusPiAndPlusPi(angle);
        BOOST_CHECK_CLOSE( cos(res), cos(angle), 0.001f);
        BOOST_CHECK_CLOSE( sin(res), sin(angle), 0.001f);
        BOOST_CHECK( res >= -PI );
        BOOST_CHECK( res <= PI );
    }
}

BOOST_AUTO_TEST_CASE( Math_firstderivate )
{
    //periode nulle
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,0,0,0,0), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,0,0,1000,1000), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(123,52,0,1000,1000), 52 );

    //acc/decc nulles
    BOOST_CHECK_EQUAL( firstDerivateLimitation(1000,0,10,0,0), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,1000,10,0,0), 1000 );

    //Cas non limités
    BOOST_CHECK_EQUAL( firstDerivateLimitation(123,0,10,-1000,1000), 123 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-123,0,10,-1000,1000), -123 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(123,-50,10,-1000,1000), 123 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-123,50,10,-1000,1000), -123 );

    //Cas limités
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1,13,27), 27 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1,-13,27), 27 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1,-13,-27), 0 );//cas erreur
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1, 13,-27), 0 );//cas erreur
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,13,27), -13 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,-13,27), -13 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,-13,-27), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,13,-27), 0 );

    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1,13,27), 87 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1,-13,27), 87 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1,-13,-27), 100 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1, 13,-27), 100 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,13,27), -73 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,-13,27), -73 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,-13,-27), -100 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,13,-27), -100 );


    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,-1,13,27), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,2,-13,27), 54 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,2,-13,27), -26 );

}

BOOST_AUTO_TEST_CASE( Math_bubbleSort )
{
    {
        Eigen::VectorXd v = arp_math::bubbleSort(Eigen::VectorXd::Random(0));
        BOOST_CHECK( v.size() == 0 );
    }
    {
        Eigen::VectorXd v0 = Eigen::VectorXd::Random(1);
        Eigen::VectorXd v = arp_math::bubbleSort(v0);
        BOOST_CHECK_EQUAL( v(0), v0(0) );

    }
    {
        Eigen::VectorXd v0 = Eigen::VectorXd::Random(2);
        Eigen::VectorXd v = arp_math::bubbleSort(v0);
        BOOST_CHECK( v(0) < v(1) );
    }
    {
        Eigen::VectorXd v = arp_math::bubbleSort(Eigen::VectorXd::Random(10));
        for(int i = 0 ; i < v.size()-1 ; i++)
        {
            BOOST_CHECK( v(i+1) >= v(i) );
        }
    }
    {
        Eigen::VectorXd v = arp_math::bubbleSort(Eigen::VectorXd::Random(17));
        for(int i = 0 ; i < v.size()-1 ; i++)
        {
            BOOST_CHECK( v(i+1) >= v(i) );
        }
    }
}

BOOST_AUTO_TEST_CASE( Math_bubbleSortIndices )
{
    {
        std::pair< Eigen::VectorXd, Eigen::VectorXi> p = arp_math::bubbleSortIndices(Eigen::VectorXd::Random(0));
        BOOST_CHECK( p.first.size() == 0 );
        BOOST_CHECK( p.second.size() == 0 );
    }
    {
        Eigen::VectorXd v0 = Eigen::VectorXd::Random(1);
        std::pair< Eigen::VectorXd, Eigen::VectorXi> p = arp_math::bubbleSortIndices(v0);
        BOOST_CHECK( p.first.size() == 1 );
        BOOST_CHECK( p.second.size() == 1 );
        BOOST_CHECK_EQUAL( p.first(0), v0(0) );
        BOOST_CHECK_EQUAL( p.second(0), 0 );

    }
    {
        Eigen::VectorXd v0 = Eigen::VectorXd::Random(2);
        std::pair< Eigen::VectorXd, Eigen::VectorXi> p = arp_math::bubbleSortIndices(v0);
        BOOST_CHECK( p.first.size() == 2 );
        BOOST_CHECK( p.second.size() == 2 );
        BOOST_CHECK( p.first(0) < p.first(1) );
        BOOST_CHECK_EQUAL( p.first(0), v0(p.second(0)) );
        BOOST_CHECK_EQUAL( p.first(1), v0(p.second(1)) );
    }
    {
        Eigen::VectorXd v0 = Eigen::VectorXd::Random(10);
        std::pair< Eigen::VectorXd, Eigen::VectorXi> p = arp_math::bubbleSortIndices(v0);
        BOOST_CHECK( p.first.size() == v0.size() );
        BOOST_CHECK( p.second.size() == v0.size() );
        for(int i = 0 ; i < p.first.size()-1 ; i++)
        {
            BOOST_CHECK( p.first(i+1) >= p.first(i) );
            BOOST_CHECK_EQUAL( p.first(i), v0(p.second(i)) );
        }
    }
    {
        Eigen::VectorXd v0 = Eigen::VectorXd::Random(17);
        std::pair< Eigen::VectorXd, Eigen::VectorXi> p = arp_math::bubbleSortIndices(v0);
        BOOST_CHECK( p.first.size() == v0.size() );
        BOOST_CHECK( p.second.size() == v0.size() );
        for(int i = 0 ; i < p.first.size()-1 ; i++)
        {
            BOOST_CHECK( p.first(i+1) >= p.first(i) );
            BOOST_CHECK_EQUAL( p.first(i), v0(p.second(i)) );
        }
    }
}

BOOST_AUTO_TEST_CASE( Math_combinaisons )
{
    std::vector< Eigen::VectorXi > res;
    res = combinaisons( Eigen::VectorXi(0), 0 );
    BOOST_CHECK( res.size() == 0 );

    {
        Eigen::VectorXi v(3);
        v << 0, 1, 2;
        res = combinaisons( v, 0 );
        BOOST_CHECK( res.size() == 0 );
    }

    {
        Eigen::VectorXi v(3);
        v << 0, 1, 2;
        res = combinaisons( v, 3 );
        BOOST_CHECK( res.size() == 1 );
    }

    {
        Eigen::VectorXi v(4);
        v << 0, 1, 2, 3, 4;
        res = combinaisons( v, 3 );
        BOOST_CHECK( res.size() == 4 );
        BOOST_CHECK_EQUAL( res[0](0), 0 );
        BOOST_CHECK_EQUAL( res[0](1), 1 );
        BOOST_CHECK_EQUAL( res[0](2), 2 );

        BOOST_CHECK_EQUAL( res[1](0), 0 );
        BOOST_CHECK_EQUAL( res[1](1), 1 );
        BOOST_CHECK_EQUAL( res[1](2), 3 );

        BOOST_CHECK_EQUAL( res[2](0), 0 );
        BOOST_CHECK_EQUAL( res[2](1), 2 );
        BOOST_CHECK_EQUAL( res[2](2), 3 );

        BOOST_CHECK_EQUAL( res[3](0), 1 );
        BOOST_CHECK_EQUAL( res[3](1), 2 );
        BOOST_CHECK_EQUAL( res[3](2), 3 );
    }

    {
        Eigen::VectorXi v(5);
        v << 0, 1, 2, 3, 4;
        res = combinaisons( v, 3 );
        BOOST_CHECK( res.size() == 10 );
    }
}

BOOST_AUTO_TEST_CASE( Math_linesIntersection )
{
    Vector2 p1(-1,0);
    Vector2 p2(1,0);
    Vector2 p3(0,-1);
    Vector2 p4(0,1);

    Vector2 result1;
    bool parralel1;
    bool colinear1;

    linesIntersection(p1,p2,p3,p4,1e-6,result1,parralel1,colinear1);

    BOOST_CHECK_SMALL(result1(0),1e-6);
    BOOST_CHECK_SMALL(result1(1),1e-6);
    BOOST_CHECK_EQUAL( parralel1, false);
    BOOST_CHECK_EQUAL( colinear1, false);

    p1=Vector2(0,2);
    p2=Vector2(2,0);
    p3=Vector2(0,0);
    p4=Vector2(2,2);

    Vector2 result2;
    bool parralel2;
    bool colinear2;

    linesIntersection(p1,p2,p3,p4,1e-6,result2,parralel2,colinear2);

    BOOST_CHECK_CLOSE( result2(0), 1.0, 1e-6 );
    BOOST_CHECK_CLOSE( result2(1), 1.0, 1e-6 );
    BOOST_CHECK_EQUAL( parralel2, false);
    BOOST_CHECK_EQUAL( colinear2, false);

    //colinear
    p1=Vector2(-1,1);
    p2=Vector2(-2,1);
    p3=Vector2(5,1);
    p4=Vector2(6,1);

    Vector2 result3;
    bool parralel3;
    bool colinear3;

    linesIntersection(p1,p2,p3,p4,1e-6,result3,parralel3,colinear3);

    BOOST_CHECK_EQUAL( parralel3, true);
    BOOST_CHECK_EQUAL( colinear3, true);


    //parralel
    p1=Vector2(-1,1);
    p2=Vector2(-1,-10);
    p3=Vector2(5,1);
    p4=Vector2(5,-20);

    Vector2 result4;
    bool parralel4;
    bool colinear4;

    linesIntersection(p1,p2,p3,p4,1e-6,result4,parralel4,colinear4);

    BOOST_CHECK_EQUAL( parralel4, true);
    BOOST_CHECK_EQUAL( colinear4, false);
}

BOOST_AUTO_TEST_CASE( Math_angleBetweenVectors )
{
    BOOST_CHECK_CLOSE( angleBetweenVectors(Vector3(1,0,0),Vector3(0,1,0)), PI/2,1e-6);
    BOOST_CHECK_CLOSE( angleBetweenVectors(Vector3(1,0,0),Vector3(-1,0,0)), PI,1e-6);
    BOOST_CHECK_CLOSE( angleBetweenVectors(Vector3(0,0,1),Vector3(0,-1,0)), PI/2,1e-6);
    BOOST_CHECK_CLOSE( angleBetweenVectors(Vector3(0,0,-1),Vector3(-sqrt(2)/2,0,-sqrt(2)/2)), PI/4,1e-6);
}



#endif /* UTEST_MATH_HPP_ */