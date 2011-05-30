/*
 * uTest_ComputeSegment.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include "ReLocalizator/ReLocalizator.hpp"
#include <math/math.hpp>

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_CASE( estimatePosition_Test )
{

    ReLocalizator rl;
    TableCorner tc;


    {
        rl.previousX = 0.0;
        rl.previousY = 0.0;
        rl.previousTheta = PI / 3.;

        TableCorner target;
        target.x = 1.0;
        target.y = 1.0;
        target.type = SOUTH_EAST;

        Corner detected;
        detected.d1 = 0.8;
        detected.alpha1 = -PI / 3.;
        detected.d2 = 0.9;
        detected.alpha2 = -PI / 3. + PI /2.;

        rl.estimatePose(detected, target);

        BOOST_CHECK_EQUAL( rl.quality, 1. );
        BOOST_CHECK_CLOSE( rl.estimatedX, 0.2, 0.01f );
        BOOST_CHECK_CLOSE( rl.estimatedY, 0.1, 0.01f );
        BOOST_CHECK_CLOSE( rl.estimatedTheta, PI / 3., 0.01f );
    }

    {
            rl.previousX = 0.15;
            rl.previousY = 0.05;
            rl.previousTheta = 3. * PI / 4.;

            TableCorner target;
            target.x = -1.0;
            target.y = 1.0;
            target.type = NORTH_EAST;

            Corner detected;
            detected.d1 = 0.8;
            detected.alpha1 = PI / 4. - PI / 2.;
            detected.d2 = 0.9;
            detected.alpha2 = PI / 4.;

            rl.estimatePose(detected, target);

            BOOST_CHECK_EQUAL( rl.quality, 1. );
            BOOST_CHECK_CLOSE( rl.estimatedX, -0.1, 0.01f );
            BOOST_CHECK_CLOSE( rl.estimatedY, 0.2, 0.01f );
            BOOST_CHECK_CLOSE( rl.estimatedTheta, 3. * PI / 4, 0.01f );
        }

}
