/*
 * PCM3362.hpp
 *
 *  Created on: 2 nov. 2010
 *      Author: ard
 */


/*TODO WLA liste des fonctions non interfacées
- SusiCoreGetCpuVendor (useless ?)
- SusiCoreGetBIOSString (useless car identique à BiosVersion)
- BOOL SusiCoreReadIO(DWORD addr, DWORD &value); (documentation inexistante)
- BOOL SusiCoreWriteIO(DWORD addr, DWORD value); (documentation inexistante)
- BOOL SusiCoreReadULongIO(DWORD addr, DWORD &value); (documentation inexistante)
- BOOL SusiCoreWriteULongIO(DWORD addr, DWORD value); (documentation inexistante)
- BOOL SusiCorePciBusSetULong(int busNum, int devNum, int funNum, UCHAR regIndex); (documentation inexistante)
- BOOL SusiCorePciBusGetULong(int busNum, int devNum, int funNum, UCHAR regIndex, DWORD &value);  (documentation inexistante)
- BOOL SusiCoreEnableBootfail(); (documentation inexistante)
- BOOL SusiCoreDisableBootfail(); (documentation inexistante)
- BOOL SusiCoreRefreshBootfail(); (documentation inexistante)

*/

#ifndef PCM3362_HPP_
#define PCM3362_HPP_

//include orocos
#include "orocos/taskcontexts/HmlTaskContext.hpp"

using namespace arp_core;


namespace arp_hml
{

    class PCM3362 : public HmlTaskContext
    {
    public:
        PCM3362(const std::string& name);

        /** Est vrai si l'initialization de Susi dans le constructeur est OK */
        bool attrSusiInitOK;

        /** Nom de la version de Susi */
        string attrSusiVersion;
        /** Nom de la carte PC */
        string attrPCBoardName;
        /** Nom de la version du BIos de la carte */
        string attrBiosVersion;

        /** Valeur minimale du timeout de Watchdog en ms */
        int attrWatchdogMinValue;
        /** Valeur maximale du timeout de Watchdog en ms */
        int attrWatchdogMaxValue;
        /** Valeur du pas du timeout de Watchdog en ms */
        int attrWatchdogStepValue;

        /** Temperature du CPU en °C*/
        float attrCpuTemperature;
        /** Temperature de la carte en °C*/
        float attrBoardTemperature;

        /** Valeur du timeout du Watchdog choisi par ARD en ms, si défini à 0 le Watchdog est désactivé */
        int propWatchdogTimer;



        /**
         * Surchargé pour :
         * - Initialiser la DLL Advantech Susi
         * - Lire le nom de la version de Susi
         * - Lire le nom de la carte PC
         */
        bool configureHook();

        /**
         * TODO WLA commenter
         */
        bool startHook();

        /**
         * Mise à jour des valeurs de temperature :
         * - attrCpuTemperature
         * - attrBoardTemperature
         */
        void updateHook();

        /**
         * Surchargé pour dé-initialiser la DLL Advantech Susi
         */
        void cleanupHook();

        /**
         * Récupère le numero de version et met à jour l'attribut attrSusiVersion
         * @return true si OK, false sinon
         */
        bool getSusiVersion();

        /**
         * Récupère le nom de la carte et met à jour l'attribut attrPCBoardName
         * @require n'appeler que si le driver Susi arp_core est disponible : SusiCoreAvailable()==true
         * @return true si OK, false sinon
         * */
        bool getPLatformName();

        /**
         * Récupère le numero du BIOS de la carte PC et met à jour l'attribut attrBiosVersion.
         * La 'BiosString' n'est pas intéressante car contient les même information que la version avec du verbe en plus
         * @require n'appeler que si le driver Susi arp_core est disponible : SusiCoreAvailable()==true
         * @return true si OK, false sinon
         * */
        bool getBiosVersion();

        /**
         * Récupère les valeurs minimales, maximales et le pas du compteur du watchdog.
         * Met à jour les attributs :
         * -attrWatchdogMaxValue
         * -attrWatchdogMinValue
         * -attrWatchdogStep
         */
        bool getWatchdogRange();

        /**
         * Pilote la configuration du watchdog.
         * @param : si activate est true, alors le Watchdog est activé avec le timeout propWatchdogTimer
         *              si activate est false, alors le Watchdog est desactivé
         * @return true si tout s'est bien passé, false sinon
         */
        bool ooManageWatchdog(bool activate);


        bool ooTriggerWatchdog();

    };

}

#endif /* PCM3362_HPP_ */
