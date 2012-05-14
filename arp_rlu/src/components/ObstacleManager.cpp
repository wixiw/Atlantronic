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
  attrFrontObstacles(),
  propMinProximity(0.1),
  propCentralZoneRadius(0.1)
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

    arp_math::Vector2 permanentObstacle(0., 0.);

    inFrontObstacles.readNewest(attrFrontObstacles);
    inRearObstacles.readNewest(attrRearObstacles);

    std::vector<arp_math::Vector2> obstacles;
    for(unsigned int i = 0 ; i < attrFrontObstacles.size() ; i++)
    {
        if( attrFrontObstacles[i].norm() < propCentralZoneRadius )
        {
            continue;
        }
        obstacles.push_back( attrFrontObstacles[i] );
    }
    for(unsigned int i = 0 ; i < attrRearObstacles.size() ; i++)
    {
        if( attrRearObstacles[i].norm() < propCentralZoneRadius )
        {
            continue;
        }
        obstacles.push_back( attrRearObstacles[i] );
    }

    std::vector<arp_math::EstimatedPose2D> opponents;
    std::vector<Eigen::VectorXi> combs = arp_math::combinaisons(obstacles.size(), 2);
    Eigen::VectorXi added = Eigen::VectorXi::Zero(obstacles.size());
    for(unsigned int i = 0 ; i < combs.size() ; i++)
    {
        if( ( (obstacles[combs[i](0)] - obstacles[combs[i](1)]).norm() < propMinProximity) && (added(combs[i](0)) == 0) && (added(combs[i](1)) == 0) )
        {
            arp_math::EstimatedPose2D opp;
            opp.x( (obstacles[combs[i](0)](0) + obstacles[combs[i](1)](0)) / 2. );
            opp.y( (obstacles[combs[i](0)](1) + obstacles[combs[i](1)](1)) / 2. );
            opp.h(0.);
            added(combs[i](0)) = 1;
            added(combs[i](1)) = 1;
            opponents.push_back( opp );
        }
        else
        {
            if(added(combs[i](0)) == 0)
            {
                arp_math::EstimatedPose2D opp;
                opp.x( obstacles[combs[i](0)](0) );
                opp.y( obstacles[combs[i](0)](1) );
                opp.h(0.);
                added(combs[i](0)) = 1;
                opponents.push_back( opp );
            }
            if(added(combs[i](1)) == 0)
            {
                arp_math::EstimatedPose2D opp;
                opp.x( obstacles[combs[i](1)](0) );
                opp.y( obstacles[combs[i](1)](1) );
                opp.h(0.);
                added(combs[i](1)) = 1;
                opponents.push_back( opp );
            }
        }
    }

    std::stringstream ss;
    ss << "***************************************************************" << std::endl;
    ss << "Obstacles (N = " << obstacles.size() << "): ";
    for(unsigned int i = 0 ; (i < obstacles.size()) ; i++)
    {
        ss << "(" << obstacles[i].transpose() << ") ";
    }
    LOG( Info ) << ss.str() << endlog();
    ss << "opponents (N = " << opponents.size() << "): ";
    for(unsigned int i = 0 ; (i < opponents.size()) ; i++)
    {
        ss << "(" << opponents[i].x() << " , " << opponents[i].y() << ") ";
    }
    LOG( Info ) << ss.str() << endlog();

    outOpponents.write(opponents);
}

void ObstacleManager::createOrocosInterface()
{
    addProperty("propNumberOfOpponents", propNumberOfOpponents);
    addProperty("propMinProximity", propMinProximity);
    addProperty("propCentralZoneRadius", propCentralZoneRadius);

    addAttribute("attrFrontObstacles", attrFrontObstacles);
    addAttribute("attrRearObstacles", attrRearObstacles);

    addPort("inFrontObstacles",inFrontObstacles)
        .doc("List of things detected with front hokuyo");

    addPort("inRearObstacles",inRearObstacles)
        .doc("List of things detected during Localization on the table");

    addPort("outOpponents",outOpponents)
        .doc("Is a vector of size propNumberOfOpponents containing opponent position");
}
