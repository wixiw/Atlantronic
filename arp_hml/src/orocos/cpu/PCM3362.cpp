/*
 * PCM3362.cpp
 *
 *  Created on: 2 nov. 2010
 *      Author: wla
 */

#include "PCM3362.hpp"
#include <ocl/Component.hpp>
#include <susi.h>

using namespace arp_hml;
using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_hml::PCM3362 )

PCM3362::PCM3362(const std::string& name) :
	HmlTaskContext(name),
    attrSusiInitOK(true),
    attrSusiVersion("Unknown"),
    attrPCBoardName("Unknown"),
    attrBiosVersion("Unknown"),
    attrWatchdogMinValue(-1),
    attrWatchdogMaxValue(-1),
    attrWatchdogStepValue(-1),
    attrCpuTemperature(-1),
    attrBoardTemperature(-1),
    propWatchdogTimer(0*1000)
{
    addAttribute("attrSusiInitOK",attrSusiInitOK);
    addAttribute("attrSusiVersion",attrSusiVersion);
    addAttribute("attrPlatformName",attrPCBoardName);
    addAttribute("attrBiosVersion",attrBiosVersion);
    addAttribute("attrWatchdogMinValue",attrWatchdogMinValue);
    addAttribute("attrWatchdogMaxValue",attrWatchdogMaxValue);
    addAttribute("attrWatchdogStepValue",attrWatchdogStepValue);
    addAttribute("attrCpuTemperature",attrCpuTemperature);
    addAttribute("attrBoardTemperature",attrBoardTemperature);

    addProperty("propWatchdogTimer",propWatchdogTimer)
        .doc("Max delay between 2 trigger on watchdog before rebooting. "
                "A value of 0 means that the watchdog is desactivated");

    addOperation("ooManageWatchdog", &PCM3362::ooManageWatchdog, this, OwnThread )
        .doc("Enable or disable Watchdog timeout. The value of the timeout is the property propWatchdogTimer")
        .arg("enable", "if set to true, the operation will enable the timeout, if set to false it will disable it");

 	addOperation("ooTriggerWatchdog", &PCM3362::ooTriggerWatchdog, this, OwnThread )
            .doc("Reset the Watchdog timeout counter");

    if ( !SusiDllInit() )
    {
        attrSusiInitOK &= false;
        LOG(Error) << "Susi initialisation failed. Error number " << SusiDllGetLastError()<< endlog();
    }
    else
    {
        //lecture de la version de Susi
        attrSusiInitOK &= getSusiVersion();


        if( SusiCoreAvailable() )
        {
            //lecture du nom de la carte PC
            attrSusiInitOK &= getPLatformName();
            //lecture du nemero de version du bios
            attrSusiInitOK &= getBiosVersion();
        }
        else
        {
            attrSusiInitOK &= false;
            LOG(Error) << "Susi Core driver is not available" << endlog();
        }

        //check de la disponibilité du watchdog
        if( SusiWDAvailable() )
        {
            //lecture des paramètres du watchdog
            attrSusiInitOK &= getWatchdogRange();
        }
        else
        {
            attrSusiInitOK &= false;
            LOG(Error) << "susi Watchdog is NOT available" << endlog();
        }
    }
}


bool PCM3362::configureHook()
{

    bool res = HmlTaskContext::configureHook();

    //si la propriété n'est pas nulle on veut mettre le watchdog en place
    if( propWatchdogTimer )
    {
        if( propWatchdogTimer < attrWatchdogMinValue || propWatchdogTimer > attrWatchdogMaxValue )
        {
            res &= false;
            LOG(Error) << "Watchdog timeout properties ouf of range. Is "
                    << propWatchdogTimer << "ms and should be in range ["
                    << attrWatchdogMinValue << ";" << attrWatchdogMaxValue << "]"
                    << endlog();
        }

        if( propWatchdogTimer%attrWatchdogStepValue )
        {
            res &= false;
            LOG(Error) << "Watchdog timeout properties is not a multiple of Hw step ("
                    << attrWatchdogStepValue << ")" << endlog();
        }
    }
    else
    {
        LOG(Info) << "Watchdog is not enabled by properties" << endlog();
    }

    return res;
}


bool PCM3362::startHook()
{
    bool res = HmlTaskContext::startHook();
//    res &= SusiWDSetConfig(3000,3000);
//    res &= SusiWDTrigger();

    return res;
}

void PCM3362::updateHook()
{
    HmlTaskContext::updateHook();

    if( SusiHWMAvailable() )
    {
        if( !SusiHWMGetTemperature(TCPU, &attrCpuTemperature, NULL) )
        {
            attrCpuTemperature = -1;
            LOG(Error) << "Cpu Temperature Read failed, error:" << SusiDllGetLastError() << endlog();
        }

        if( !SusiHWMGetTemperature(TCPU, &attrBoardTemperature, NULL) )
        {
            attrCpuTemperature = -1;
            LOG(Error) << "Board Temperature Read failed, error:" << SusiDllGetLastError() << endlog();
        }
    }
    else
    {
        attrCpuTemperature = -1;
        attrBoardTemperature = -1;
        LOG(Error) << "Susi Hardware NOT available, error:" << SusiDllGetLastError() << endlog();
    }

//    if( SusiWDAvailable() )
//    {
//        if( !SusiWDTrigger() )
//        {
//            LOG(Info) << "Trigger WD" << endl;
//        }
//    }
//    else
//    {
//        LOG(Warning) << "Susi Watchdog available" << endlog();
//    }

    LOG(Info) << "Update" << endl;
}


void PCM3362::cleanupHook()
{
    HmlTaskContext::cleanupHook();

    if ( !SusiDllUnInit() )
    {
        LOG(Error) << "Susi un-initialisation failed" << endlog();
    }

    HmlTaskContext::cleanupHook();
}


bool PCM3362::getSusiVersion()
{
    bool res = true;
    DWORD BUF_LENGTH = 128;
    DWORD major, minor, year, month, date;
    char bufferVersionName[BUF_LENGTH];

    SusiDllGetVersion(&major, &minor);
    year    = minor/10000;
    month   = minor%10000/100;
    date    = minor%100;
    sprintf(bufferVersionName, "V%li (20%02li/%02li/%02li)" , major, year, month, date);
    attrSusiVersion = bufferVersionName;
    if( attrSusiVersion != "V3 (2010/11/25)" )
    {
        res &= false;
        LOG(Error) << "Susi version unexpected : " << attrSusiVersion << endlog();
    }

    return res;
}


bool PCM3362::getPLatformName()
{
    bool res = true;
    DWORD BUF_LENGTH = 128;
    TCHAR bufferPcName[BUF_LENGTH];

    if( SusiCoreGetPlatformName(bufferPcName, &BUF_LENGTH) )
    {
        attrPCBoardName = bufferPcName;
        if( attrPCBoardName != "PCM-3362" )
        {
            res &= false;
            LOG(Error) << "PC board name unexpected : " << attrPCBoardName << endlog();
        }
    }
    else
    {
        res &= false;
        LOG(Error) << "PC board name read error " << SusiDllGetLastError() << endlog();
    }

    return res;
}


bool PCM3362::getBiosVersion()
{
    bool res = true;

    DWORD BUF_LENGTH = 128;
    TCHAR bufferBiosVersion[BUF_LENGTH];

    if( SusiCoreGetBIOSVersion(bufferBiosVersion, &BUF_LENGTH) )
    {
        attrBiosVersion = bufferBiosVersion;
        if( attrBiosVersion != "V1.11 (03/29/2010)" )
        {
            res &= false;
            LOG(Error) << "Bios board version unexpected : " << attrBiosVersion << endlog();
        }
    }
    else
    {
        res &= false;
        LOG(Error) << "Bios board version read error " << SusiDllGetLastError() << endlog();
    }

    return res;
}


bool PCM3362::getWatchdogRange()
{
    bool res = true;

    DWORD min = 0;
    DWORD max = 0;
    DWORD step = 0;

    if( SusiWDGetRange(&min,&max,&step) )
    {
        attrWatchdogMinValue = min;
        attrWatchdogMaxValue = max;
        attrWatchdogStepValue = step;
    }
    else
    {
        res &= false;
        LOG(Error) << "Watchdog Range read error " << SusiDllGetLastError() << endlog();
    }

    return res;
}


bool PCM3362::ooManageWatchdog(bool activate)
{
    bool res = true;

    if( SusiWDAvailable() )
    {
        if( activate )
        {
            if( propWatchdogTimer )
            {
                //premier argument = timeout différent en attendant le premier Trigger pour laisser au soft le temps de s'initiliser
                //deuxieme argument = timeout en opérationnel
                res &= SusiWDSetConfig( 10000 , 10000 );
                if( !res )
                {
                    LOG(Warning) << "Enable Watchdog failed with error code :" << SusiDllGetLastError() << endl;
                }
            }
            else
            {
                res &= false;
                LOG(Warning) << "Watchdog activation was tried but properties desactivated it "  << endlog();
            }
        }
        else
        {
            res &= SusiWDDisable();
            if( !res )
            {
                LOG(Warning) << "Disable  Watchdogfailed with error code :" << SusiDllGetLastError() << endl;
            }
        }
    }
    else
    {
        res &= false;
        LOG(Warning) << "Susi Watchdog section is not available" << endl;
    }

    return res;
}

bool PCM3362::ooTriggerWatchdog()
{
    bool res = true;

    if( !SusiWDTrigger() )
    {
        res &= false;
        LOG(Critical) << "Watchdog trigger error " << SusiDllGetLastError() << endlog();
    }

    return res;
}

