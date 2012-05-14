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

    /* Params */
    int propNumberOfOpponents;

    std::vector<arp_math::Vector2> attrInObstacles;

    /* Ports */
    RTT::InputPort< std::vector<arp_math::Vector2> > inObstacles;
    RTT::OutputPort< std::vector<arp_math::EstimatedPose2D> > outOpponents;

    void createOrocosInterface();


};

} /* namespace arp_rlu */
#endif /* ObstacleManager_HPP_ */
