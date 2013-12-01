/*
 * ARDTaskContext.cpp
 *
 *  Created on: 8 oct. 2010
 *      Author: ard
 */

#include "ARDTaskContext.hpp"
#include "math/core"

using namespace arp_core;
using namespace RTT;
using namespace base;
using namespace std;
using namespace arp_math;

ARDTaskContext::ARDTaskContext(const std::string& name, const std::string projectRootPath) :
                TaskContext(name, PreOperational),
                propEnableLog(true),
                propAutoLoadScript(""),
                propAutoLoadStateMachines(""),
                propTimeReporting(false),
                attrProjectRootPath(projectRootPath),
                attrPropertyPath("script/orocos/conf"),
                attrScriptPath("script/orocos/ops"),
                attrStateMachinePath("script/orocos/osd"),
                attrDt(0.0),
                /* TODO WLA logger(dynamic_cast<OCL::logging::Category*>(&log4cpp::Category::getInstance("ard")))*/
                logger(name),
                m_timer(0)
{
    addProperty("propAutoLoadScript", propAutoLoadScript).doc(
            "If non empty, the component will automatically load this script during configure() (le dossier ops est automatiquement ajouté, l'extension est mise par le soft)");
    addProperty("propAutoLoadStateMachines", propAutoLoadStateMachines).doc(
            "If non empty, the component will automatically load this state machine during configure() (le dossier fsm-osd est automatiquement ajouté, l'extension est mise par le soft)");
    addProperty("propEnableLog", propEnableLog).doc(
            "If set to true the composant is allowed to write in the log file, if set to false it will not log anything. Use it when a component is spamming log and this annoys you");
    addProperty("propTimeReporting",propTimeReporting).doc(
            "Activate or not Time statistics computation");

    addAttribute("attrProjectRootPath", attrProjectRootPath);
    addAttribute("attrPropertyPath", attrPropertyPath);
    addAttribute("attrScriptPath", attrScriptPath);
    addAttribute("attrStateMachinePath", attrStateMachinePath);
    addAttribute("attrScriptRes", attrScriptRes);
    addAttribute("attrUpdateTime", attrUpdateTime);
    addAttribute("attrDt", attrDt);

    addOperation("coLog", &ARDTaskContext::coLog, this, ClientThread).doc(
            "This Client Operation allows to log from script. It's non real time. Level are NONE=-1,ERROR=0,WARNING=1,INFO=2,DEBUG=3").arg(
                    "level", "The log level").arg("string", "The string to log");
    addOperation("coReset", &ARDTaskContext::coReset, this, ClientThread).doc(
            "This Client Operation allows reset the component");
    addOperation("ooWriteProperties", &ARDTaskContext::ooWriteProperties, this, OwnThread).doc(
            "Write current Component properties to disk");

    addOperation("ooGetPerformanceReport", &ARDTaskContext::ooGetPerformanceReport, this, OwnThread).doc("Get Performance report about timing, already formated in readable string");
    addOperation("ooSetMaxBufferSize", &ARDTaskContext::ooSetMaxBufferSize, this, OwnThread)
        .arg(
                  "bufSize",
                  "New number of period that will be logged.")
        .doc("Change the timing log buffer size.");

    attrUpdateTime.tv_sec = 0.0;
    attrUpdateTime.tv_nsec = 0.0;

}

ARDTaskContext::~ARDTaskContext()
{
}

//------------------------------------------------------------------------------------------------------------------

bool ARDTaskContext::configureHook()
{
    bool res = TaskContext::configureHook();

    if ( /* TODO logger==NULL*/false)
    {
        res &= false;
        cerr << "Logger is badly initialised '" << "'" << endl;
    }
    else
    {
        res &= loadPlugins();
        res &= loadProperties();
        res &= loadPrograms();
        res &= loadStateMachines();
    }

    return res;
}

bool ARDTaskContext::startHook()
{
    bool res = TaskContext::startHook();

    m_timer.ResetStat();

    return res;
}

void ARDTaskContext::updateHook()
{
    timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);

    if (attrUpdateTime.tv_nsec == 0.0 && attrUpdateTime.tv_sec == 0.0)
    {
        //first time here
        attrDt = 0.0;
        attrUpdateTime = time;
    }
    else
    {
        attrDt = delta_t(attrUpdateTime,time);
        attrUpdateTime = time;
    }

    TaskContext::updateHook();
}

void ARDTaskContext::stopHook()
{
    TaskContext::stopHook();
}

void ARDTaskContext::cleanupHook()
{
    string fileName;
    std::vector<std::string> progList = scripting->getProgramList();
    std::vector<std::string>::iterator itProg;
    std::vector<std::string> fsmList = scripting->getStateMachineList();
    std::vector<std::string>::iterator itFsm;

    //dechargement de tous les programs
    for (itProg = progList.begin(); itProg != progList.end(); itProg++)
    {
        scripting->unloadProgram((*itProg));
    }

    //dechargement de toutes les states machines
    for (itFsm = fsmList.begin(); itFsm != fsmList.end(); itFsm++)
    {
        scripting->unloadStateMachine((*itFsm));
    }

    TaskContext::cleanupHook();
}

//------------------------------------------------------------------------------------------------------------------

bool ARDTaskContext::loadPlugins()
{
    bool res = true;

    //ajout du plugin de scripting
    scripting = this->getProvider<RTT::Scripting>("scripting");
    if (scripting == NULL)
    {
        LOG(Error) << "Interface de scripting non chargée" << endlog();
        res &= false;
    }

    //ajout du plugin de marshalling
    marshalling = this->getProvider<RTT::Marshalling>("marshalling");
    if (marshalling == NULL)
    {
        LOG(Error) << "Interface de marschalling non chargée" << endlog();
        res &= false;
    }

    return res;
}

bool ARDTaskContext::loadProperties()
{
    bool res = true;
    //nom du fichier à rechercher
    string fileName = attrProjectRootPath + "/" + attrPropertyPath + "/" + getName() + ".xml";

    if (marshalling != NULL)
    {
        if (std::ifstream(fileName.c_str()))
        {
            if (marshalling->updateProperties(fileName) == false)
            {
                LOG(Error) << "Configure failed, error while reading " << fileName << endlog();
                res &= false;
            }
            //on a reussit à charger les propriétés !
            else
            {
                LOG(Info) << "Properties loaded successfully from " << fileName << endlog();
                res &= true;
            }
        }
    }
    else
    {
        LOG(Error) << "Marshalling is not ready" << endlog();
        res &= false;
    }

    //vérification de la valeur des propriétés
    if (res)
    {
        res &= checkProperties();
    }

    return res;
}

bool ARDTaskContext::loadPrograms()
{
    bool res = true;
    string path;

    if (propAutoLoadScript == "")
    {
        LOG(Info) << "no auto loaded script" << endlog();
        res &= true;
    }
    else
    {
        path = attrProjectRootPath + "/" + attrScriptPath + "/" + propAutoLoadScript + ".ops";

        //on tente de charger le programme, on renvoit une erreur si on ne le trouve pas
        res &= scripting->loadPrograms(path);
    }

    return res;
}

bool ARDTaskContext::loadStateMachines()
{
    bool res = true;
    string path;

    //si la propriété est vide on ne charge rien
    if (propAutoLoadStateMachines == "")
    {
        LOG(Info) << "no auto loaded fsm" << endlog();
        res &= true;
    }
    else
    {
        path = attrProjectRootPath + "/" + attrStateMachinePath + "/" + propAutoLoadStateMachines + ".osd";

        //on tente de charger la state machine, on renvoit une erreur si on ne le trouve pas
        res &= scripting->loadStateMachines(path);
    }

    return res;
}

void ARDTaskContext::coLog(LoggerLevel level, string s)
{
    LOG(level) << s << endlog();
}

bool ARDTaskContext::checkProperties()
{
    return true;
}

//TODO WOUAT ? Est-ce vérifié ailleurs comme dans le deployer par ex ?
//bool ARDTaskContext::checkInputsPorts()
//{
//    bool res = true;
//    DataFlowInterface::Ports portsVector = ports()->getPorts();
//    DataFlowInterface::Ports::iterator it;
//    string name;
//
//    for(it = portsVector.begin(); it != portsVector.end(); it++)
//    {
//        if( dynamic_cast<InputPortInterface*>(*it) != 0 && (*it)->connected() == false )
//        {
//            LOG(Error)  << "checkInputsPorts : " << (*it)->getName() << " is not connected !" << endlog();
//            res &= false;
//        }
//
//    }
//
//    return res;
//}

bool ARDTaskContext::coReset()
{
    bool res = true;

    LOG(Info) << "ooReset in progress..." << endlog();

    stop();
    cleanup();
    res &= configure();
    res &= start();

    if (!res)
    {
        LOG(Error) << "Failed to reset" << endlog();
    }

    return res;
}

bool ARDTaskContext::ooWriteProperties()
{
    bool res = true;

    //vérification de la valeur des propriétés
    if (checkProperties() == false)
    {
        LOG(Warning) << "ooWriteProperties : checkProperties failed ! Properties saved may be unloadable next time"
                << endlog();
        ;
    }

    //nom du fichier à rechercher
    string fileName = attrProjectRootPath + "/" + attrPropertyPath + "/" + getName() + ".xml";

    if (marshalling != NULL)
    {
        if (marshalling->writeProperties(fileName) == false)
        {
            LOG(Error) << "ooWriteProperties : error while writing properties in file " << fileName << endlog();
            res &= false;
        }
        //on a reussit à charger les propriétés !
        else
        {
            LOG(Info) << "ooWriteProperties : Properties saved successfully from " << fileName << endlog();
            res &= true;
        }
    }
    else
    {
        LOG(Error) << "coWriteProperties : Marshalling is not ready" << endlog();
        res &= false;
    }

    return res;
}

void ARDTaskContext::ooGetPerformanceReport()
{
    if( !isRunning() || !propTimeReporting )
        cout << "Time Stats are disabled. The component must be in running state with propTimereporting=true." << endl;
    else
        cout << m_timer.GetReport() << endl;
}

void ARDTaskContext::ooSetMaxBufferSize(unsigned int size)
{
    m_timer.SetMaxBufferSize(size);
}
