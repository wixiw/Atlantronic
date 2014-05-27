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
    attrActionDone = isActionDone();
    attrStucked = isArmStucked();

    mutex.unlock();

    ArmCommandMsg armCmdMsg;
    if( RTT::NewData == inArmCommand.read(armCmdMsg) )
    {
        attrLastCommand = armCmdMsg.command;
        sendCmd(attrLastCommand);
    }

    outArmPosition.write(m_armMatrix);

    ArmStatusMsg armStatusMsg;
    armStatusMsg.action_done = attrActionDone;
    armStatusMsg.stucked = attrStucked;

    outArmStatus.write(armStatusMsg);
}

bool Arm::isActionDone()
{
    //TODO JB : a remplir
    return false;
}

bool Arm::isArmStucked()
{
    //TODO JB : a remplir
    return true;
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
    addAttribute("attrLastCommand", attrLastCommand);
    addAttribute("attrActionDone", attrActionDone);
    addAttribute("attrStucked", attrStucked);

    addPort("outArmPosition", outArmPosition);
    addPort("outArmStatus", outArmStatus);
    addPort("inArmCommand", inArmCommand);

}
