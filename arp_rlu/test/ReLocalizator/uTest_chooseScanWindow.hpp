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

BOOST_AUTO_TEST_CASE( chooseScanWindow_Test )
{

    ReLocalizator rl;
    TableCorner tc;
    std::pair<double, double> p;

    {
        rl.previousX = 0.0;
        rl.previousY = 0.0;
        rl.previousTheta = 0.0;

        tc.x = 1.0;
        tc.y = 1.0;
        tc.type = SOUTH_EAST;
        std::vector<TableCorner> vtc;
        vtc.push_back(tc);

        rl.setTableCorners(vtc);
        rl.printTableCorners();
        TableCorner target = rl.selectTargetTableCorner();

        p = rl.chooseScanWindow(target);

        BOOST_CHECK_CLOSE( p.first, betweenMinusPiAndPlusPi(atan2(tc.y - 0.25, tc.x) - rl.previousTheta), 0.01f );
        BOOST_CHECK_CLOSE( p.second, betweenMinusPiAndPlusPi(atan2(tc.y, tc.x - 0.25) - rl.previousTheta), 0.01f );
    }

    {
        rl.previousX = 0.0;
        rl.previousY = 0.0;
        rl.previousTheta = PI/3.;

        tc.x = 1.0;
        tc.y = 1.0;
        tc.type = SOUTH_EAST;
        std::vector<TableCorner> vtc;
        vtc.push_back(tc);

        rl.setTableCorners(vtc);
        rl.printTableCorners();
        TableCorner target = rl.selectTargetTableCorner();

        p = rl.chooseScanWindow(target);

        BOOST_CHECK_CLOSE( p.first, betweenMinusPiAndPlusPi(atan2(tc.y - 0.25, tc.x) - rl.previousTheta), 0.01f );
        BOOST_CHECK_CLOSE( p.second, betweenMinusPiAndPlusPi(atan2(tc.y, tc.x - 0.25) - rl.previousTheta), 0.01f );
    }

    {
        rl.previousX = 0.0;
        rl.previousY = 0.0;
        rl.previousTheta = 0.0;

        tc.x = 0.8;
        tc.y = -1.0;
        tc.type = SOUTH_WEST;
        std::vector<TableCorner> vtc;
        vtc.push_back(tc);

        rl.setTableCorners(vtc);
        rl.printTableCorners();
        TableCorner target = rl.selectTargetTableCorner();

        p = rl.chooseScanWindow(target);

        BOOST_CHECK_CLOSE( p.first, betweenMinusPiAndPlusPi(atan2(tc.y, tc.x - 0.25) - rl.previousTheta), 0.01f );
        BOOST_CHECK_CLOSE( p.second, betweenMinusPiAndPlusPi(atan2(tc.y + 0.25, tc.x) - rl.previousTheta), 0.01f );
    }

    //    BOOST_CHECK_EQUAL( tc.y, 1.0 );
    //    BOOST_CHECK_EQUAL( tc.type, SOUTH_EAST );

}
