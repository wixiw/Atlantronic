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
  attrFrontObstacles()
{
    createOrocosInterface();
    std::vector<arp_math::EstimatedPose2D> opponents(propNumberOfOpponents);
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

    inFrontObstacles.readNewest(attrFrontObstacles);
    inRearObstacles.readNewest(attrRearObstacles);

    std::stringstream ss;
    ss << "***************************************************************" << std::endl;
    ss << "Obstacles (N = " << attrFrontObstacles.size() <<  "+" << attrRearObstacles.size() << "): ";
    for(unsigned int i = 0 ; (i < attrFrontObstacles.size()) && ( i < 3) ; i++)
    {
        ss << "(" << attrFrontObstacles[i].transpose() << ") ";
    }
    for(unsigned int i = 0 ; (i < attrRearObstacles.size()) && ( i < 3) ; i++)
    {
        ss << "(" << attrRearObstacles[i].transpose() << ") ";
    }
    LOG( Info ) << ss.str() << endlog();

    opponents.push_back(Pose2D(-0.500,0.500,0.0));
    outOpponents.write(opponents);
}

void ObstacleManager::createOrocosInterface()
{
    addProperty("propNumberOfOpponents", propNumberOfOpponents);

    addAttribute("attrFrontObstacles", attrFrontObstacles);
    addAttribute("attrRearObstacles", attrRearObstacles);

    addPort("inFrontObstacles",inFrontObstacles)
        .doc("List of things detected with front hokuyo");

    addPort("inRearObstacles",inRearObstacles)
        .doc("List of things detected during Localization on the table");

    addPort("outOpponents",outOpponents)
        .doc("Is a vector of size propNumberOfOpponents containing opponent position");
}
