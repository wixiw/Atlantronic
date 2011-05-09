/*
 * ARDTaskContext.cpp
 *
 *  Created on: 8 oct. 2010
 *      Author: ard
 */

#include "ARDTaskContext.hpp"
#include <iostream>
#include <rtt/base/InputPortInterface.hpp>

using namespace arp_core;
using namespace RTT;
using namespace base;

ARDTaskContext::ARDTaskContext(const std::string& name, const std::string projectRootPath) :
    TaskContext(name, PreOperational),
    propAutoLoadScript(""),
    propAutoLoadStateMachines(""),
    propEnableLog(true),
    attrProjectRootPath(projectRootPath),
    attrPropertyPath("script/orocos/conf"),
    attrScriptPath("script/orocos/ops"),
    attrStateMachinePath("script/orocos/osd"),
    /* TODO logger(dynamic_cast<OCL::logging::Category*>(&log4cpp::Category::getInstance("ard")))*/
    logger(name)
{
    addProperty("propAutoLoadScript", propAutoLoadScript)
        .doc("If non empty, the component will automatically load this script during configure() (le dossier ops est automatiquement ajouté, l'extension est mise par le soft)");
    addProperty("propAutoLoadStateMachines",propAutoLoadStateMachines)
        .doc("If non empty, the component will automatically load this state machine during configure() (le dossier fsm-osd est automatiquement ajouté, l'extension est mise par le soft)");
    addProperty("propEnableLog", propEnableLog)
        .doc("If set to true the composant is allowed to write in the log file, if set to false it will not log anything. Use it when a component is spamming log and this annoys you");

    addAttribute("attrPropertyPath",attrPropertyPath);
    addAttribute("attrScriptPath",attrScriptPath);
    addAttribute("attrStateMachinePath",attrStateMachinePath);

    addOperation("coLog", &ARDTaskContext::coLog, this, ClientThread )
        .doc("This Client Operation allows to log from script. It's non real time. Level are NONE=-1,ERROR=0,WARNING=1,INFO=2,DEBUG=3")
        .arg("level","The log level")
        .arg("string","The string to log");
    addOperation("ooReset", &ARDTaskContext::ooReset, this, OwnThread )
        .doc("This Client Operation allows reset the component");
    addOperation("ooWriteProperties", &ARDTaskContext::ooWriteProperties, this, OwnThread )
          .doc("Write current Component properties to disk");

}

ARDTaskContext::~ARDTaskContext()
{
}

//------------------------------------------------------------------------------------------------------------------

bool ARDTaskContext::configureHook()
{
    bool res = TaskContext::configureHook();

    if ( /* TODO logger==NULL*/ false )
    {
        res &= false;
        cerr << "Logger is badly initialised '"  << "'" << endl;
    }
    else
    {
        res &= loadPlugins();
        res &= loadProperties();
        res &= loadPrograms();
        res &= loadStateMachines();
    }

    res &= checkInputsPorts();

    return res;
}

bool ARDTaskContext::startHook()
{
    bool res = TaskContext::startHook();

    return res;
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

//   if( marshalling != NULL )
//   {
//       fileName = attrRootProjectPath + "/" + attrPropertyPath  + "/" + getName() + ".xml";
//       marshalling->writeProperties(fileName);
//   }

   //dechargement de tous les programs
   for( itProg = progList.begin() ; itProg != progList.end() ; itProg++ )
   {
       scripting->unloadProgram((*itProg));
   }

   //dechargement de toutes les states machines
   for( itFsm = fsmList.begin() ; itFsm != fsmList.end() ; itFsm++ )
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
    if( scripting == NULL )
    {
        LOG(Error) << "Interface de scripting non chargée" << endlog();
        res &= false;
    }

    //ajout du plugin de marshalling
    marshalling = this->getProvider<RTT::Marshalling>("marshalling");
    if( marshalling == NULL )
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
    string fileName = attrProjectRootPath + "/" + attrPropertyPath  + "/" + getName() + ".xml";

    if( marshalling != NULL)
    {
        if( marshalling->readProperties(fileName) == false )
        {
            LOG(Error) << "Configure failed, error while reading or file not found " << fileName << endlog();
            res &= false;
        }
        //on a reussit à charger les propriétés !
        else
        {
            LOG(Info) << "Properties loaded successfully from " << fileName << endlog();
            res &= true;
        }
    }
    else
    {
        LOG(Error) << "Marshalling is not ready" << endlog();
        res &= false;
    }

    //vérification de la valeur des propriétés
    if( res )
    {
         res &= checkProperties();
    }

    return res;
}

bool ARDTaskContext::loadPrograms()
{
    bool res    = true;
    string path ;

    if( propAutoLoadScript == "" )
    {
        LOG(Info) << "no auto loaded script" << endlog();
        res &= true;
    }
    else
    {
        path = attrProjectRootPath + "/" + attrScriptPath + "/" +propAutoLoadScript + ".ops";

        //on tente de charger le programme, on renvoit une erreur si on ne le trouve pas
        res &= scripting->loadPrograms(path);
    }

    return res;
}

bool ARDTaskContext::loadStateMachines()
{
    bool res    = true;
    string path ;

    //si la propriété est vide on ne charge rien
    if( propAutoLoadStateMachines == "" )
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

bool ARDTaskContext::coLog(int level, string s)
{
    LoggerLevel logLevel;

    switch(level)
    {
    case 0:
        logLevel = Error;
        break;
    case 1:
        logLevel = Warning;
        break;
    case 2:
        logLevel = Info;
        break;
    case 3:
        logLevel = Debug;
        break;

    default:
        //par defaut on met Error parce que ça apparait tout le temps, sauf pour
        logLevel = Error;
        break;
    }

    //si le level est -1 on ne veut plus logger
    if( level != -1 )
    {
        LOG(logLevel) << s << endlog();
    }
}


bool ARDTaskContext::checkProperties()
{
    return true;
}

bool ARDTaskContext::checkInputsPorts()
{
    bool res = true;
    DataFlowInterface::Ports portsVector = ports()->getPorts();
    DataFlowInterface::Ports::iterator it;
    string name;

    for(it = portsVector.begin(); it != portsVector.end(); it++)
    {
        if( dynamic_cast<InputPortInterface*>(*it) != 0 && (*it)->connected() == false )
        {
            LOG(Error)  << "checkInputsPorts : " << (*it)->getName() << " is not connected !" << endlog();
            res &= false;
        }

    }

    return res;
}

bool ARDTaskContext::ooReset()
{
    bool res = true;

    LOG(Info) << "ooReset in progress..." << endlog();

    stop();
    cleanup();
    res &= configure();
    res &= start();

    if( !res )
    {
        LOG(Error) << "Failed to reset" << endlog();
    }

    return res;
}

bool ARDTaskContext::ooWriteProperties()
{
    bool res = true;


    //vérification de la valeur des propriétés
    if( checkProperties() == false )
    {
        LOG(Warning) << "coWriteProperties : checkProperties failed ! Properties saved may be unloadable next time" << endlog();;
    }

    //nom du fichier à rechercher
    string fileName = attrProjectRootPath + "/" + attrPropertyPath  + "/" + getName() + ".xml";

    if( marshalling != NULL)
    {
        if( marshalling->writeProperties(fileName) == false )
        {
            LOG(Error) << "coWriteProperties : error while writing properties in file " << fileName << endlog();
            res &= false;
        }
        //on a reussit à charger les propriétés !
        else
        {
            LOG(Info) << "coWriteProperties : Properties saved successfully from " << fileName << endlog();
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
