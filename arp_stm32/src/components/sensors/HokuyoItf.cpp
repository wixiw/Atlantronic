#include "components/sensors/HokuyoItf.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std_msgs;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::HokuyoItf)

HokuyoItf::HokuyoItf(const std::string& name) :
        Stm32TaskContext(name),
        propHokuyoId(HOKUYO_MAX),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
}

bool HokuyoItf::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void HokuyoItf::updateHook()
{
    Stm32TaskContext::updateHook();
    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    if(propHokuyoId < 0 || propHokuyoId >= HOKUYO_MAX)
    {
        LOG(Error) << "propHokuyoId is out of range" << endlog();
        stop();
    }
    else
    {
        outScan.write(m_robotItf.hokuyo_scan[propHokuyoId]);
    }

    std::vector<arp_math::Vector2> opponents;



    for( int i=0 ; i < getNbDetectedObjects() ; i++)
    {
        struct detection_object* obstacles = getObstacles();
        if( obstacles == NULL )
        {
            LOG(Error) << "Failed to find obstacle list due du index errors." << endlog();
            stop();
            return;
        }

        /*LOG(Info) << "count= " << m_robotItf.detection_dynamic_object_count
                << " i=" << i << " x="<<m_robotItf.detection_obj[i].x/1000.0
                << " y" << m_robotItf.detection_obj[i].y/1000.0 << endlog();*/
        opponents.push_back( arp_math::Vector2(
                obstacles[i].x/1000.0,
                obstacles[i].y/1000.0));
    }

    mutex.unlock();

    outObstacles.write(opponents);
}

struct detection_object* HokuyoItf::getObstacles()
{
    if( propHokuyoId == HOKUYO_AVANT)
    {
        return m_robotItf.detection_obj1;
    }
    if( propHokuyoId == HOKUYO_ARRIERE)
    {
        return m_robotItf.detection_obj2;
    }
    return NULL;
}


int HokuyoItf::getNbDetectedObjects()
{
    int nbObjects = 0;
    if( propHokuyoId == HOKUYO_AVANT)
    {
        nbObjects = m_robotItf.detection_dynamic_object_count1;
    }
    else if( propHokuyoId == HOKUYO_ARRIERE)
    {
        nbObjects = m_robotItf.detection_dynamic_object_count2;
    }

    return nbObjects;
}


void HokuyoItf::createOrocosInterface()
{
    addPort("outScan",outScan);
    addPort("outObstacles",outObstacles);

    addProperty("propHokuyoId",propHokuyoId);
}

