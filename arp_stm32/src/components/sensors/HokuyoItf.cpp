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
        m_robotItf(DiscoveryMutex::robotItf),
        propHokuyoId(HOKUYO_MAX)
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
    LOG(Info) << "updateHook" <<  endlog();
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

    mutex.unlock();
}

void HokuyoItf::createOrocosInterface()
{
    addPort("outScan",outScan);

    addProperty("propHokuyoId",propHokuyoId);
}

