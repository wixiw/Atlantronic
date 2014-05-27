/*
 * ObstacleManager.hpp
 *
 *  Created on: 14 mai 2012
 *      Author: ard
 */

#ifndef ObstacleManager_HPP_
#define ObstacleManager_HPP_

#include "RluTaskContext.hpp"
#include <math/core>


namespace arp_rlu
{

class ObstacleManager: public RluTaskContext
{
    public:
    ObstacleManager(const std::string& name);
    bool configureHook();
    void updateHook();

    protected:

    bool isObstacleOutOfScope(arp_math::Vector2 obstacle);

    /* Params */
    int propNumberOfOpponents;
    double propMinProximity;

    std::vector<arp_math::Vector2> attrFrontObstacles;
    std::vector<arp_math::Vector2> attrRearObstacles;

    /* Ports */
    RTT::InputPort< std::vector<arp_math::Vector2> > inFrontObstacles;
    RTT::InputPort< std::vector<arp_math::Vector2> > inRearObstacles;
    RTT::OutputPort< std::vector<arp_math::EstimatedPose2D> > outOpponents;

    void createOrocosInterface();


};

} /* namespace arp_rlu */
#endif /* ObstacleManager_HPP_ */
