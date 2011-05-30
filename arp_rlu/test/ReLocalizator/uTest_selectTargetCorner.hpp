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

BOOST_AUTO_TEST_CASE( selectTargetCorner_Test )
{

    ReLocalizator rl;
    rl.previousX = 0.0;
    rl.previousY = 0.0;
    rl.previousTheta = PI / 2.;

    std::vector<TableCorner> vtc;
    TableCorner tc;


    tc.x = 1.0;
    tc.y = 1.0;
    tc.type = SOUTH_WEST;
    vtc.push_back(tc);

    tc.x = 2.0;
    tc.y = 2.0;
    tc.type = SOUTH_EAST;
    vtc.push_back(tc);

    tc.x = -1.0;
    tc.y = -1.0;
    tc.type = NORTH_WEST;
    vtc.push_back(tc);

    tc.x = 1.0;
    tc.y = 1.0;
    tc.type = SOUTH_EAST;
    vtc.push_back(tc);

    tc.x = -1.0;
    tc.y = -1.0;
    tc.type = SOUTH_WEST;
    vtc.push_back(tc);

    rl.setTableCorners(vtc);
    rl.printTableCorners();
    tc = rl.selectTargetTableCorner();

    std::cout << "Selected Corner :" << std::endl;
    std::cout << "  x = " << tc.x << std::endl;
    std::cout << "  y = " << tc.y << std::endl;
    switch (tc.type)
    {
        case NORTH_WEST:
            std::cout << "  type : NORTH_WEST" << std::endl;
            break;
        case NORTH_EAST:
            std::cout << "  type : NORTH_EAST" << std::endl;
            break;
        case SOUTH_EAST:
            std::cout << "  type : SOUTH_EAST" << std::endl;
            break;
        case SOUTH_WEST:
            std::cout << "  type : SOUTH_WEST" << std::endl;
            break;
        default:
            std::cout << "  type : NONE" << std::endl;
            break;
    }

    BOOST_CHECK_EQUAL( tc.x, 1.0 );
    BOOST_CHECK_EQUAL( tc.y, 1.0 );
    BOOST_CHECK_EQUAL( tc.type, SOUTH_EAST );

}
