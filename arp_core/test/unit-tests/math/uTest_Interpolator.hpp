/*
 * uTest_Interpolator.hpp
 *
 *  Created on: 23 February 2012
 *      Author: boris
 */

#ifndef _UTEST_INTERPOLATOR_HPP_
#define _UTEST_INTERPOLATOR_HPP_

#include "math/Interpolator.hpp"
using namespace arp_math;
using namespace Eigen;


BOOST_AUTO_TEST_SUITE( unittest_Interpolator )

BOOST_AUTO_TEST_CASE( find_N0 )
{
    VectorXd v(0);
    int ret = Interpolator::find(0.1, v);
    BOOST_CHECK_EQUAL( ret, -1 );
}

BOOST_AUTO_TEST_CASE( find_N1 )
{
    VectorXd v(1);
    v << 2.0;
    int ret = Interpolator::find(0.1, v);
    BOOST_CHECK_EQUAL( ret, -1 );

    ret = Interpolator::find(3.0, v);
    BOOST_CHECK_EQUAL( ret, 0 );

    ret = Interpolator::find(2.0, v);
    BOOST_CHECK_EQUAL( ret, 0 );
}

BOOST_AUTO_TEST_CASE( find_N5 )
{
    VectorXd v(5);
    v << 2.0, 2.5, 4.0, 5.1, 6.0;
    int ret;

    ret = Interpolator::find(0.1, v);
    BOOST_CHECK_EQUAL( ret, -1 );

    ret = Interpolator::find(1.99, v);
    BOOST_CHECK_EQUAL( ret, -1 );

    ret = Interpolator::find(2.0, v);
    BOOST_CHECK_EQUAL( ret, 0 );

    ret = Interpolator::find(2.01, v);
    BOOST_CHECK_EQUAL( ret, 0 );

    ret = Interpolator::find(2.49, v);
    BOOST_CHECK_EQUAL( ret, 0 );

    ret = Interpolator::find(2.5, v);
    BOOST_CHECK_EQUAL( ret, 1 );

    ret = Interpolator::find(4.5, v);
    BOOST_CHECK_EQUAL( ret, 2 );

    ret = Interpolator::find(6.0, v);
    BOOST_CHECK_EQUAL( ret, 4 );

    ret = Interpolator::find(6.01, v);
    BOOST_CHECK_EQUAL( ret, 4 );
}

BOOST_AUTO_TEST_CASE( transInterp_N_different_P )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(5);
    VectorXd yyc = VectorXd::Random(6);

    VectorXd result = Interpolator::transInterp(tt, ttc, yyc);
    BOOST_CHECK_EQUAL( result.size(), 0 );
}

BOOST_AUTO_TEST_CASE( transInterp_N0 )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(0);
    VectorXd yyc = VectorXd::Random(0);

    VectorXd result = Interpolator::transInterp(tt, ttc, yyc);
    BOOST_CHECK_EQUAL( result.size(), 0 );
}

BOOST_AUTO_TEST_CASE( transInterp_N1 )
{
    VectorXd tt  = VectorXd::Random(3);  // P = 3
    VectorXd ttc = VectorXd::Random(1);  // N = 1
    VectorXd yyc = VectorXd::Random(1);  // N = 1

    VectorXd result = Interpolator::transInterp(tt, ttc, yyc);
    BOOST_CHECK_EQUAL( result.size(), 3 );
    for(unsigned int i = 0 ; i < tt.size() ; i++)
    {
        BOOST_CHECK_EQUAL( result[i], yyc[0] );
    }
}

BOOST_AUTO_TEST_CASE( transInterp_N3_P7 )
{
    VectorXd tt(7);
    tt << 0.0, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0;
    VectorXd ttc(3);
    ttc << 1.0, 2.0, 3.0;
    VectorXd yyc(3);
    yyc << 0.0, 1.0, 3.0;

    VectorXd result = Interpolator::transInterp(tt, ttc, yyc);
    BOOST_CHECK_EQUAL( result.size(), tt.size() );

    BOOST_CHECK_EQUAL( result[0], -1.0 ); //t = 0.0
    BOOST_CHECK_EQUAL( result[1], 0.0 );  //t = 1.0
    BOOST_CHECK_EQUAL( result[2], 0.5 );  //t = 1.5
    BOOST_CHECK_EQUAL( result[3], 1.0 );  //t = 2.0
    BOOST_CHECK_EQUAL( result[4], 2.0 );     //t = 2.5
    BOOST_CHECK_EQUAL( result[5], 3.0 );     //t = 3.0
    BOOST_CHECK_EQUAL( result[6], 5.0 );     //t = 4.0
}

BOOST_AUTO_TEST_CASE( rotInterp_N_different_P )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(5);
    VectorXd yyc = VectorXd::Random(6);

    VectorXd result = Interpolator::rotInterp(tt, ttc, yyc);
    BOOST_CHECK_EQUAL( result.size(), 0 );
}

BOOST_AUTO_TEST_CASE( rotInterp_N0 )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(0);
    VectorXd yyc = VectorXd::Random(0);

    VectorXd result = Interpolator::rotInterp(tt, ttc, yyc);
    BOOST_CHECK_EQUAL( result.size(), 0 );
}

BOOST_AUTO_TEST_CASE( rotInterp_N1 )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(1);
    VectorXd yyc = VectorXd::Random(1);

    VectorXd result = Interpolator::rotInterp(tt, ttc, yyc);
    BOOST_CHECK_EQUAL( result.size(), 3 );
    for(unsigned int i = 0 ; i < tt.size() ; i++)
    {
        BOOST_CHECK_EQUAL( result[i], yyc[0] );
    }
}

BOOST_AUTO_TEST_CASE( rotInterp_N3_P7_1 )
{
    VectorXd tt(7);
    tt << -1.0, 0.0, 1.0, 2.0, 2.5, 3.0, 4.0;
    VectorXd ttc(3);
    ttc << 0.0, 2.0, 3.0;
    VectorXd hhc(3);
    hhc << 0.0, PI/2., PI;

    VectorXd result = Interpolator::rotInterp(tt, ttc, hhc);
    BOOST_CHECK_EQUAL( result.size(), tt.size() );

    BOOST_CHECK_EQUAL( result[0], -PI/4.);   //t = -1.0
    BOOST_CHECK_EQUAL( result[1], 0.0 );     //t = 0.0
    BOOST_CHECK_EQUAL( result[2], PI/4.);    //t = 1.0
    BOOST_CHECK_EQUAL( result[3], PI/2.);    //t = 2.0
    BOOST_CHECK_EQUAL( result[4], 3.*PI/4.); //t = 2.5
    BOOST_CHECK_EQUAL( result[5], PI);       //t = 3.0
    BOOST_CHECK_EQUAL( result[6], -PI/2. );  //t = 4.0
}

BOOST_AUTO_TEST_CASE( rotInterp_N3_P7_2 )
{
    VectorXd tt(7);
    tt << -1.0, 0.0, 1.0, 2.0, 2.5, 3.0, 4.0;
    VectorXd ttc(3);
    ttc << 0.0, 2.0, 3.0;
    VectorXd hhc(3);
    hhc << 3.*PI/4., -3.*PI/4., 0.;

    VectorXd result = Interpolator::rotInterp(tt, ttc, hhc);
    BOOST_CHECK_EQUAL( result.size(), tt.size() );

    BOOST_CHECK_EQUAL( result[0], PI/2.);    //t = -1.0
    BOOST_CHECK_EQUAL( result[1], 3.*PI/4.); //t = 0.0
    BOOST_CHECK_EQUAL( result[2], PI);       //t = 1.0
    BOOST_CHECK_EQUAL( result[3],-3.*PI/4.); //t = 2.0
    BOOST_CHECK_EQUAL( result[4],-3.*PI/8.); //t = 2.5
    BOOST_CHECK_EQUAL( result[5], 0.0);      //t = 3.0
    BOOST_CHECK_EQUAL( result[6], 3.*PI/4.); //t = 4.0
}

BOOST_AUTO_TEST_CASE( covInterp_N_different_P )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(5);
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covc(6);

    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > result = Interpolator::covInterp(tt, ttc, covc);
    BOOST_CHECK_EQUAL( result.size(), 0 );
}

BOOST_AUTO_TEST_CASE( covInterp_N0 )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(0);
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covc(0);

    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > result = Interpolator::covInterp(tt, ttc, covc);
    BOOST_CHECK_EQUAL( result.size(), 0 );
}

BOOST_AUTO_TEST_CASE( covInterp_N1 )
{
    VectorXd tt  = VectorXd::Random(3);
    VectorXd ttc = VectorXd::Random(1);
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covc(1);
    for(unsigned int k = 0 ; k < covc.size() ; k++)
    {
        covc[k] = Eigen::Matrix3d::Random();
        for(unsigned int i = 0 ; i < 3 ; i++)
        {
            for(unsigned int j = i ; j < 3 ; j++)
            {
                double m = (covc[k](i,j) + covc[k](j,i)) / 2.;
//                covc[k](i,j) = m > 0. ? m : 0.;
                covc[k](i,j) = m;
                covc[k](j,i) = covc[k](i,j);
            }
        }
    }

    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > result = Interpolator::covInterp(tt, ttc, covc);
    BOOST_CHECK_EQUAL( result.size(), 3 );
    for(unsigned int k = 0 ; k < tt.size() ; k++)
    {
        for(unsigned int i = 0 ; i < 3 ; i++)
        {
            for(unsigned int j = 0 ; j < 3 ; j++)
            {
                BOOST_CHECK_EQUAL( result[k](i,j), covc[0](i,j) );
            }
        }
    }
}

BOOST_AUTO_TEST_CASE( covInterp_N3_P5_1 )
{
    VectorXd tt(5);
    tt << -0.5, 0.0, 0.5, 1.0, 1.5;
    VectorXd ttc(2);
    ttc << 0.0, 1.0;
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covc(2);
    covc(0) << 1.0, 1.0, 3.0,
               1.0, 2.0, 0.0,
               3.0, 0.0, 2.0;

    covc(1) << 1.0, 0.0, 1.0,
               0.0, 3.0, 0.0,
               1.0, 0.0, 8.0;

    double epsilon = 1e-6;
    Eigen::Vector3d minimum = Eigen::Vector3d(1e-12,1e-12,1e-9);


    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > solutions(5);
    solutions(0) << 1.0, 1.5, 4.0,
                    1.5, 1.5, 0.0,
                    4.0, 0.0, minimum(2);

    solutions(1) << 1.0, 1.0, 3.0,
                    1.0, 2.0, 0.0,
                    3.0, 0.0, 2.0;

    solutions(2) << 1.0, 0.5, 2.0,
                    0.5, 2.5, 0.0,
                    2.0, 0.0, 5.0;

    solutions(3) << 1.0, 0.0, 1.0,
                    0.0, 3.0, 0.0,
                    1.0, 0.0, 8.0;

    solutions(4) << 1.0,-0.5, 0.0,
                   -0.5, 3.5, 0.0,
                    0.0, 0.0,11.0;

    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > result = Interpolator::covInterp(tt, ttc, covc, epsilon, minimum);
    BOOST_CHECK_EQUAL( result.size(), tt.size() );

    for(unsigned int k = 0 ; k < tt.size() ; k++)
    {
        for(unsigned int i = 0 ; i < 3 ; i++)
        {
            for(unsigned int j = 0 ; j < 3 ; j++)
            {
                if(j == i)
                {
                    BOOST_CHECK( result[k](i,j) > 0. );
                }
                else
                {
                    BOOST_CHECK_EQUAL( result[k](i,j), result[k](j,i) );
                }
                BOOST_CHECK_EQUAL( result[k](i,j), solutions[k](i,j) );
            }
        }
    }

}

BOOST_AUTO_TEST_SUITE_END()

#endif /* _UTEST_INTERPOLATOR_HPP_ */
