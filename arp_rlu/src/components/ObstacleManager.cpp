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
  propMinProximity(0.1),
  propActivateFakeRobots(false),
  propFakeLittleRobot(-0.400, 0.400, 0.0),
  propFakeBigRobot( 0.000, -0.500, 0.0),
  attrFrontObstacles(),
  attrRearObstacles()
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
    arp_math::Vector2 permanentObstacle(0., 0.);

    inFrontObstacles.readNewest(attrFrontObstacles);
    inRearObstacles.readNewest(attrRearObstacles);

    std::vector<arp_math::Vector2> obstacles;
    for(unsigned int i = 0 ; i < attrFrontObstacles.size() ; i++)
    {
        if( isObstacleOutOfScope(attrFrontObstacles[i]) )
        {
            continue;
        }
        obstacles.push_back( attrFrontObstacles[i] );
    }
    for(unsigned int i = 0 ; i < attrRearObstacles.size() ; i++)
    {
        if( isObstacleOutOfScope(attrFrontObstacles[i]) )
        {
            continue;
        }
        obstacles.push_back( attrRearObstacles[i] );
    }

//    LOG( Info ) << "***************************************************************" << endlog();
//    {
//        std::stringstream ss;
//        ss << "Obstacles (N = " << obstacles.size() << "): ";
//        for(unsigned int i = 0 ; (i < obstacles.size()) ; i++)
//        {
//            ss << "(" << obstacles[i].transpose() << ") ";
//        }
//
//        LOG( Info ) << ss.str() << endlog();
//    }

    std::vector<arp_math::EstimatedPose2D> opponents;

    while(true)
    {
        if( obstacles.size() < 2 )
        {
            break;
        }

        std::vector<Eigen::VectorXi> combs = arp_math::combinaisons(obstacles.size(), 2);
//        {
//            std::stringstream ss;
//            ss << "combs : ";
//            for(unsigned int i = 0 ; i < combs.size() ; i++)
//            {
//                ss << "(" << combs[i].transpose() << ") ";
//            }
//            LOG( Info ) << ss.str() << endlog();
//        }

        Eigen::VectorXd dist = Eigen::VectorXd::Ones(combs.size()) * propMinProximity * 2.;

        for(unsigned int i = 0 ; i < combs.size() ; i++)
        {
            dist(i) = (obstacles[combs[i](0)] - obstacles[combs[i](1)]).norm();
        }
        unsigned int iMin;
        double dmin = dist.minCoeff(&iMin);
        if( dmin < propMinProximity )
        {
            std::vector<arp_math::Vector2> newObstacles;
            for(int i = 0 ; i < obstacles.size() ; i++)
            {
                if( (i != combs[iMin](0)) && (i != combs[iMin](1)) )
                {
                    newObstacles.push_back(obstacles[i]);
                }
            }
            arp_math::Vector2 obs = (obstacles[combs[iMin](0)] + obstacles[combs[iMin](1)]) / 2.;
            newObstacles.push_back( obs  );
            obstacles = newObstacles;
//            LOG( Info ) << "Merge " << combs[iMin](0) << " and " << combs[iMin](1) << endlog();
            continue;
        }
        else
        {
            break;
        }
    }

    for(unsigned int i = 0 ; i < obstacles.size() ; i++)
    {
        arp_math::EstimatedPose2D opp;
        opp.x( obstacles[i](0) );
        opp.y( obstacles[i](1) );
        opp.h(0.);
        opponents.push_back( opp );
    }

//    {
//        std::stringstream ss;
//        ss << "opponents (N = " << opponents.size() << "): ";
//        for(unsigned int i = 0 ; (i < opponents.size()) ; i++)
//        {
//            ss << "(" << opponents[i].x() << " , " << opponents[i].y() << ") ";
//        }
//        LOG( Info ) << ss.str() << endlog();
//    }

    //TODO Willy : fake object to test avoidance system
    if( propActivateFakeRobots )
    {
        opponents.clear();
        arp_math::EstimatedPose2D oppDummy(propFakeLittleRobot);
        arp_math::EstimatedPose2D oppDummy2(propFakeBigRobot);
        opponents.push_back( oppDummy );
        opponents.push_back( oppDummy2 );
    }

    outOpponents.write(opponents);
}

bool ObstacleManager::isObstacleOutOfScope(Vector2 obstacle)
{
    //TODO filtrer les arbres
    return false;
}

void ObstacleManager::createOrocosInterface()
{
    addProperty("propNumberOfOpponents", propNumberOfOpponents);
    addProperty("propMinProximity", propMinProximity);
    addProperty("propActivateFakeRobots", propActivateFakeRobots);
    addProperty("propFakeLittleRobot", propFakeLittleRobot);
    addProperty("propFakeBigRobot", propFakeBigRobot);

    addAttribute("attrFrontObstacles", attrFrontObstacles);
    addAttribute("attrRearObstacles", attrRearObstacles);

    addEventPort("inFrontObstacles",inFrontObstacles)
    .doc("List of things detected with front hokuyo");

    addEventPort("inRearObstacles",inRearObstacles)
    .doc("List of things detected during Localization on the table");

    addPort("outOpponents",outOpponents)
    .doc("Is a vector of size propNumberOfOpponents containing opponent position");
}
