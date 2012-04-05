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

BOOST_AUTO_TEST_CASE( TableCorner_CompatibleCorner_Test )
{
    double xLaser = 1.0;
    double yLaser = 0.8;
    double minAngle = 0.0;
    double maxAngle = 0.0;

    TableCorner tc;
    bool visible;

    tc.x = 1.3;
    tc.y = 1.2;
    tc.type = NORTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = NORTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = SOUTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == true );
    tc.type = SOUTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );

    tc.x = 0.7;
    tc.y = 1.2;
    tc.type = NORTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = NORTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == true );
    tc.type = SOUTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = SOUTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );

    tc.x = 0.7;
    tc.y = 0.4;
    tc.type = NORTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == true );
    tc.type = NORTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = SOUTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = SOUTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );

    tc.x = 1.5;
    tc.y = 0.4;
    tc.type = NORTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = NORTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = SOUTH_EAST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
    tc.type = SOUTH_WEST;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == true );

}

BOOST_AUTO_TEST_CASE( TableCorner_ANgularLimits_Test )
{
    double xLaser = 0.0;
    double yLaser = 0.0;

    TableCorner tc;
    bool visible;
    double minAngle;
    double maxAngle;

    tc.x = 1.0;
    tc.y = 1.0;
    tc.type = SOUTH_EAST;
    minAngle = 0.;
    maxAngle = PI / 2.;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == true );

    minAngle = PI / 4. + 0.01;
    maxAngle = PI / 2.;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );

    minAngle = 0.;
    maxAngle = PI / 4. - 0.01;
    visible = tc.isVisibleFrom(xLaser, yLaser, minAngle, maxAngle);
    BOOST_CHECK( visible == false );
}
