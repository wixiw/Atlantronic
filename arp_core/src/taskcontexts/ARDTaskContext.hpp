/*
 * ARDTaskContext.hpp
 *
 *  Created on: 8 oct. 2010
 *      Author: wla
 */

#ifndef ARDTASKCONTEXT_HPP_
#define ARDTASKCONTEXT_HPP_

#include <iostream>
#include <fstream>

//accelerateur de compilation
#include <rtt/typekit/Types.hpp>

#include <rtt/RTT.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <timer/StatTimer.hpp>

namespace arp_core
{
/** surcharge du log */
#define LOG(level) RTT::log(propEnableLog?level:RTT::Never)<<"["<<getName()<<"] "

/**
 * Create a while loop to wait a function to answer true until a timeout explodes
 * YOU MUST provide in your code a "double chrono=0.0;" variable !
 * @param function : a boolean function that returns true when the loop must continue
 * @param timeout : a maximal time in s
 * @param sleep : sleeping delay each time function is true in s
 */
#define whileTimeout(function, timeout, sleep ) \
        while( function && chrono >= 0 && chrono < timeout )       \
        {                                           \
            chrono += sleep;                        \
            usleep(sleep*1E6);                      \
        }

/**
 * Use this define after a whileTimeout to check
 * if the timeout has expired
 * use it like a if else case :
 * IfWhileTimeoutExpired
 * {
 *    LOG(Error) << "Timeout over !" << endlog();
 * }
 * else
 * {
 *    LOG(Info) << "Great ! it worked." << endlog();
 * }
 */
#define IfWhileTimeoutExpired(timeout)  if( chrono >= timeout )



/** \ingroup ARP-arp_core
 *
 * \class ARDTaskContext
 *
 * Cette classe contient l'instrumentation de base utile
 * pour tous les composants qui seron développés avec Orocos
 */
class ARDTaskContext : public RTT::TaskContext
{
    public:

    /** Constructeur pour définir le chemin vers le projet. Utile pour ROS*/
    ARDTaskContext(const std::string& name, const std::string projectRootPath=".");
    /** Destructeur par défaut */
    virtual ~ARDTaskContext();


    /** Defini si le composant est autorisé à logger. Publique pour que les services puissent y accéder */
    bool propEnableLog;

    /** Allows a component to connect easily to an other component's operation */
    template<class CommandT>
    bool getOperation(std::string component, std::string operation, RTT::OperationCaller<CommandT>& reference)
    {
            bool res = true;
            RTT::TaskContext* taskContext;
            RTT::OperationInterfacePart* opItf;

            //si je cherche une operation sur moi c'est facile, sinon je cherche le peer
            if( getName() == component )
            {
                taskContext = this;
            }
            else
            {
                taskContext = getPeer(component);
            }

            if ( taskContext == NULL )
            {
                LOG(RTT::Error)  << "getOperation : failed to find peer " << component  << " (when asking for " << operation << RTT::endlog();
                res = false;
            }
            else
            {
                opItf = taskContext->getOperation(operation);
                if( opItf == NULL || opItf->getLocalOperation() == NULL )
                {
                    LOG(RTT::Error)  << "getOperation : " << component << "." << operation << " is not available" << RTT::endlog();
                    res = false;
                }
                else
                {
                    reference = opItf;
                }
            }

            return res;
    }

    protected:

    /** Nom du script à charger automatiquement, si vide on ne charge rien snas erreur */
    std::string propAutoLoadScript;
    /** Nom de la machine à états à charger automatiquement, si vide on ne charge rien snas erreur */
    std::string propAutoLoadStateMachines;
    /** Activate or not the time reporting */
    bool propTimeReporting;

    /**Chemin vers le project */
    std::string attrProjectRootPath;
    /** Chemin vers le dossier des propriétés depuis attrProjectRootPath  */
    std::string attrPropertyPath;
    /** Chemin vers le dossier des scripts depuis attrProjectRootPath */
    std::string attrScriptPath;
    /** Chemin vers le dossier des fsm depuis attrProjectRootPath */
    std::string attrStateMachinePath;
    /** Used by configuration programs to set a success flag */
    bool attrScriptRes;
    /** cycle time*/
    timespec attrUpdateTime;
    /** cycle duration*/
    double attrDt;

    /** Interface de scripting Orocos (ods,ops) */
    boost::shared_ptr<RTT::Scripting> scripting;

    /** Interface de marshalling XML (sauvegarde des propriétés)  */
    boost::shared_ptr<RTT::Marshalling> marshalling;

    /** Logger */
    // TODO WLA OCL::logging::Category* logger;
    RTT::Logger::In logger;

    arp_core::StatTimer m_timer;

    /** Callback de Configuration.*/
    virtual bool configureHook();
    /** Callback de Démarrage */
    virtual bool startHook();
    /** Calcule le temps de cycle (attention au premier cycle la période est nulle */
    virtual void updateHook();
    /** Callback d'Arrêt */
    virtual void stopHook();
    /** Callback de Déconfiguration */
    virtual void cleanupHook();

    /**
     * Check if the properties have correct values. This function may be override in components
     */
    virtual bool checkProperties();

    /*
     * DEPRECATED : this job is done by the monitoring component
     *
     * Check if the input ports have been connected
     * It gets all the input ports automatically and check if they are connected
     * If this is undesired behavior please override the function
     */
    //virtual bool checkInputsPorts();

    /**
     * Reset the component in doing in chain : stop();cleanup();configure();start()
     * @return true if the component is back to a running state
     */
    bool coReset();

    /**
     * Write current Component properties to disk
     * @return true if the component is back to a running state
     */
    bool ooWriteProperties();

    /** Charge les plugins du composant, à appeler dans le configureHook */
    virtual bool loadPlugins();
    /** Charge un fichier de propriété au format xml */
    virtual bool loadProperties();
    /** Charge le script qui porte le nom du composant s'il existe */
    virtual bool loadPrograms();
    /** Charge la state machine qui porte le nom du programme si elle existe */
    virtual bool loadStateMachines();

    /** Permet de logger en script */
    void coLog(RTT::LoggerLevel level, std::string s);

    /**
     * Permet d'obtenir un rapport sur les timings
     */
    virtual void ooGetPerformanceReport();

    /**
     * Permet de regler la duree pendant laquelle les stats de performances temporelles sont faites
     */
    void ooSetMaxBufferSize(unsigned int size);
};
}

#endif /* ARDTASKCONTEXT_HPP_ */
