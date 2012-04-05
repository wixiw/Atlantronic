/*
 * uTest_Split.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include "cornerdetector/CornerDetector.hpp"
#include <math/math.hpp>

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_CASE( Split_Test )
{
    srand(time(0));

    for(unsigned i = 0; i < 100; i++)
    {
        CornerDetector cd;

        double d1 = 0.1 + (double) rand() / RAND_MAX;
        double d2 = 0.1 + (double) rand() / RAND_MAX;
        double alpha1 = (double) rand() / RAND_MAX * 2. * PI;
        double alpha2 = alpha1 + PI / 2.;
        double angleBegin = alpha1 - PI / 8. * ( 1. + (double) rand() / RAND_MAX ) ;
        double angleEnd = alpha1 + PI / 2. + PI / 8. * ( 1. + (double) rand() / RAND_MAX ) ;
        alpha1 = fmod( alpha1 + 4. * PI, 2. * PI);
        alpha2 = fmod( alpha2 + 4. * PI, 2. * PI);
        double nbMeas = 200;

        Scan polarScan(2, nbMeas);
        for (unsigned int i = 0; i < nbMeas; i++)
        {
            double angle = fmod( angleBegin + i * (angleEnd - angleBegin) / (nbMeas - 1) , 2.*PI );
            double d = d1;
            double alpha = alpha1;
            if( i > nbMeas / 2)
            {
                d = d2;
                alpha = alpha2;
            }
            polarScan(0, i) = angle;
            polarScan(1, i) = d / cos(alpha - angle);
        }
        angleBegin = fmod( angleBegin + 4. * PI, 2. * PI);
        angleEnd = fmod( angleEnd + 4. * PI, 2. * PI);

        std::vector<Segment> sgmts = cd.split(polarScan, 0.01, 2);

        BOOST_CHECK_EQUAL( sgmts.size(), 2 );
        BOOST_CHECK_CLOSE( sgmts[0].d, d1, 1.f );
        BOOST_CHECK_CLOSE( sgmts[0].alpha, alpha1, 1.f );
        if( sgmts.size() == 2 )
        {
            BOOST_CHECK_CLOSE( sgmts[1].d, d2, 1.f );
            BOOST_CHECK_CLOSE( sgmts[1].alpha, alpha2, 1.f );
        }
        else
        {
            std::cout << "d1:" << d1 << std::endl;
            std::cout << "alpha1:" << alpha1 << std::endl;
            std::cout << "d2:" << d2 << std::endl;
            std::cout << "alpha2:" << alpha2 << std::endl;
            std::cout << "****************************************" << std::endl;
        }
    }
}
