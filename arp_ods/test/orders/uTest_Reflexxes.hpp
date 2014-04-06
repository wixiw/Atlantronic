/*
 * uTest_FantomOrder.hpp
 *
 *  Created on: 24 may 2011
 *      Author: wla
 */

#include "control/orders/orders.h"
#include "math/math.hpp"
using namespace arp_ods;
using namespace arp_math;

BOOST_AUTO_TEST_CASE( ReflexxesTrajectory1 )
{
    // Test default constructor
    OnlineTrajectoryGenerator OTG;

    PosVelAcc start;
    PosVelAcc end;
    PosVelAcc next;
    double maxSpeed;
    double maxAcc;
    double maxJerk;
/*
    start.position=0.0262057;
    start.velocity=-0.0387513;
    start.acceleration=-1.94073;

    end.position=0.308722;
    end.velocity=1;
    end.acceleration=0;
*/

    start.position=0;
    start.velocity=+0.0387513;
    start.acceleration=+1.94073;

    end.position=-(0.308722-0.0262057);
    end.velocity=-1;
    end.acceleration=0;


    maxSpeed=1;
    maxAcc=2;
    maxJerk=10;

    bool OTGres=false;

    cout << "TimeStamp NextPosition NextVelocity NextAcceleration" << endl;

    double timeStamp=0;

    for (int i=0;i<100*10;i++)
    {
    OTGres = OTG.computeNextStep(start, end, maxSpeed, maxAcc, maxJerk, next);

    cout << timeStamp<<" "<< next.position <<" "<< next.velocity<<" "<< next.acceleration << endl;
    start=next;
    timeStamp+=0.010;
    }


}

