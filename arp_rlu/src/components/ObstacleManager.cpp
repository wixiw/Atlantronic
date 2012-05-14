/*
 * ObstacleManager.cpp
 *
 *  Created on: 14 mai 2012
 *      Author: ard
 */

#include <iomanip>

#include "ObstacleManager.hpp"
#include <rtt/Component.hpp>

using namespace arp_rlu;
using namespace arp_math;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::ObstacleManager )

ObstacleManager::ObstacleManager(const std::string& name)
: RluTaskContext(name),
  propNumberOfOpponents(2),
  attrInObstacles()
{
    createOrocosInterface();
    std::vector<arp_math::EstimatedPose2D> opponents(2);
    outOpponents.write(opponents);
}

bool ObstacleManager::configureHook()
{
    if( !RluTaskContext::configureHook() )
        return false;

    return true;
}


void ObstacleManager::updateHook()
{
    RluTaskContext::updateHook();
    std::vector<arp_math::EstimatedPose2D> opponents;
    std::vector<arp_math::EstimatedPose2D>::iterator opp;

    inObstacles.readNewest(attrInObstacles);
    //TODO
   // bla bla bla ... il faut peupler opponents à partir de arrtInObstacles
   //en attendant y'a un adversaire fixe

    opponents.push_back(Pose2D(-0.500,0.500,0.0));
    outOpponents.write(opponents);
}

void ObstacleManager::createOrocosInterface()
{
    addProperty("propNumberOfOpponents", propNumberOfOpponents);

    addAttribute("attrInObstacles", attrInObstacles);

    addPort("inObstacles",inObstacles)
        .doc("List of things detected during Localization on the table");

    addPort("outOpponents",outOpponents)
        .doc("Is a vector of size propNumberOfOpponents containing opponent position");
}
