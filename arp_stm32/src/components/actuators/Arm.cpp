#include "components/actuators/Arm.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_core;
using namespace std_msgs;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Arm)

using namespace arp_stm32;


Arm::Arm(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
}

bool Arm::configureHook()
{
    if (!Stm32TaskContext::configureHook())
    {
        return false;
    }

    return true;
}

void Arm::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    m_armMatrix = m_robotItf.last_control_usb_data.arm_matrix;

    mutex.unlock();

    outArmPosition.write(m_armMatrix);
}

void Arm::sendCmd(uint32_t cmdType)
{
    int errorCode = m_robotItf.arm_cmd(cmdType);
    if (errorCode < 0)
    {
        LOG(Error) << "Failed to send arm cmd" << endlog();
    }
}


void Arm::createOrocosInterface()
{

}
