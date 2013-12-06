/*
 * test_RnLPButterworthFilter.hpp
 *
 *  Created on: 14 mai 2012
 *      Author: boris
 */


#include <filters/RnLPButterworthFilter.hpp>
#include <iostream>
#include <fstream>
using namespace std;
using namespace arp_math;

BOOST_AUTO_TEST_CASE( RnLPButterworthFilter_Parameters )
{
    arp_math::RnLPButterworthFilter<double, 3> filter(0.020, 5.);

    BOOST_CHECK_EQUAL( filter.getSamplingPeriod(), 0.020);
    BOOST_CHECK_EQUAL( filter.getCutOffFrequency(), 5.);

    filter.setParameters(0.050, 15.);

    BOOST_CHECK_EQUAL( filter.getSamplingPeriod(), 0.050);
    BOOST_CHECK_EQUAL( filter.getCutOffFrequency(), 15.);
}

BOOST_AUTO_TEST_CASE( RnLPButterworthFilter_Test1 )
{
    arp_math::RnLPButterworthFilter<double, 3> filter(0.020, 5.);

    Eigen::Vector3d X(1., 2., 3.);
    filter.initialize( X );

    BOOST_CHECK( filter.isInitialized() );

    filter.set_x_raw(X);
    filter.compute();
    Eigen::Vector3d X_filtered = filter.get_x_filtered();

    BOOST_CHECK_EQUAL( X_filtered(0), X(0) );
    BOOST_CHECK_EQUAL( X_filtered(1), X(1) );
    BOOST_CHECK_EQUAL( X_filtered(2), X(2) );
}

BOOST_AUTO_TEST_CASE( RnLPButterworthFilter_Test2 )
{
    double T = 0.020;
    double fc = 5.;
    arp_math::RnLPButterworthFilter<double, 3> filter(T, fc);

    Eigen::Vector3d X(0., 0., 0.);
    filter.initialize( X );

    BOOST_CHECK( filter.isInitialized() );

    std::ofstream myfile;
    std::string fileName = "/tmp/LPButterworth.data";
    myfile.open(fileName.c_str());
    myfile << "Timestamp X X_filtered" << std::endl;

    Eigen::Vector3d X_filtered;

    double f1 = 0.6;
    double f2 = 20.;
    //unsigned int N = 10000;
    for(unsigned int i = 0 ; i < 100 ; i++)
    {
        X(0)  = sin( 2. * M_PI * f1 * i * T);
        X(0) += sin( 2. * M_PI * f2 * i * T);
        filter.set_x_raw(X);
        filter.compute();
        X_filtered = filter.get_x_filtered();
        myfile << (double)i*T << " " << X(0) << " " << X_filtered(0) << std::endl;
    }
    myfile.close();
//
//    BOOST_CHECK_SMALL( X_filtered(0) - sin( 2. * M_PI * f2 * (N-1) * T), 0.05 );
//    BOOST_CHECK_EQUAL( X_filtered(1), 0. );
//    BOOST_CHECK_EQUAL( X_filtered(2), 0. );
}
