/*
 * ARDTaskContext.hpp
 *
 *  Created on: 8 oct. 2010
 *      Author: wla
 */

#ifndef ARDTASKCONTEXT_HPP_
#define ARDTASKCONTEXT_HPP_

//TODO WLA a remettre plus tard, bug Orocos
//#include <rtt/typekit/Types.hpp>
#include <rtt/RTT.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>


using namespace RTT;

namespace arp_core
{
    /** surcharge du log */
    #define LOG(level) RTT::log(propEnableLog?level:Never)<<"["<<getName()<<"] "

    /**
     * Create a while loop to wait a function to answer true until a timeout explodes
     * YOU MUST provide in your code a "double chrono=0.0;" variable !
     * @param function : a boolean function that returns true when the loop must continue
     * @param timeout : a maximal time in s
     * @param sleep : sleeping delay each time function is true in s
     */
    #define whileTimeout(function, timeout, sleep ) \
    while( function && chrono < timeout )       \
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
    class ARDTaskContext : public TaskContext
    {
    public:

        /** Constructeur pour définir le chemin vers le projet. Utile pour ROS*/
        ARDTaskContext(const std::string& name, const std::string projectRootPath=".");
        /** Destructeur par défaut */
        virtual ~ARDTaskContext();

        /** Callback de Configuration.*/
        virtual bool configureHook();
        /** Callback de Démarrage */
        virtual bool startHook();
        /** Callback d'Arrêt */
        virtual void stopHook();
        /** Callback de Déconfiguration */
        virtual void cleanupHook();

        /** Defini si le composant est autorisé à logger. Publique pour que les services puissent y accéder */
        bool propEnableLog;

    protected:

        /** Nom du script à charger automatiquement, si vide on ne charge rien snas erreur */
        string propAutoLoadScript;
        /** Nom de la machine à états à charger automatiquement, si vide on ne charge rien snas erreur */
        string propAutoLoadStateMachines;

        /**Chemin vers le project */
        string attrProjectRootPath;
        /** Chemin vers le dossier des propriétés depuis attrProjectRootPath  */
        string attrPropertyPath;
        /** Chemin vers le dossier des scripts depuis attrProjectRootPath */
        string attrScriptPath;
        /** Chemin vers le dossier des fsm depuis attrProjectRootPath */
        string attrStateMachinePath;
        /** Used by configuration programs to set a success flag */
        bool attrScriptRes;

        /** Interface de scripting Orocos (ods,ops) */
        boost::shared_ptr<Scripting> scripting;

        /** Interface de marshalling XML (sauvegarde des propriétés)  */
        boost::shared_ptr<Marshalling> marshalling;

        /** Logger */
        // TODO WLA OCL::logging::Category* logger;
        Logger::In logger;

        /** Allows a component to connect easily to an other component's operation */
        template<class CommandT>
        bool getOperation(string component, string operation, OperationCaller<CommandT>& reference)
        {
            bool res = true;

            TaskContext* taskContext = getPeer(component);
            if ( taskContext == NULL )
            {
                LOG(Error)  << "getOperation : failed to find peer " << component << endlog();
                res = false;
            }
            else
            {
                reference = taskContext->getOperation(operation);
                //TODO WLA je ne sais pas comment on check que le get a reussi !
//                if( reference. )
//                {
//                    LOG(Error)  << "getOperation : " << component << "." << operation << " is not available" << endlog();
//                    res = false;
//                }
            }

            return res;
        }

        /**
         * Check if the properties have correct values. This function may be override in components
         */
        virtual bool checkProperties();

        /**
         * Check if the input ports have been connected
         * It gets all the input ports automatically and check if they are connected
         * If this is undesired behavior please override the function
         */
        virtual bool checkInputsPorts();

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

    private:

        /** Charge les plugins du composant, à appeler dans le configureHook */
        bool loadPlugins();
        /** Charge un fichier de propriété au format xml */
        bool loadProperties();
        /** Charge le script qui porte le nom du composant s'il existe */
        bool loadPrograms();
        /** Charge la state machine qui porte le nom du programme si elle existe */
        bool loadStateMachines();

        /** Permet de logger en script */
        bool coLog(LoggerLevel level, string s);


    };
}

#endif /* ARDTASKCONTEXT_HPP_ */
