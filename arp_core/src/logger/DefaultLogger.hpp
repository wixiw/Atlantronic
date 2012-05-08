/*
 * Logger.hpp
 *
 *  Created on: 09 May 2012
 *      Author: Willy
 */

#ifndef _ARP_CORE_DEFAULT_LOGGER_HPP_
#define _ARP_CORE_DEFAULT_LOGGER_HPP_

#include "LoggerSystem.hpp"

namespace arp_core
{

namespace log
{

/** \class Logger
 *
 * \brief Logger une classe a surcharger puis instancier dans une lib pour avoir un logger. C'est un singleton.
 */
class DefaultLogger
{
    public:
        /** InitNull initialise le Logger pour qu'il publie dans aucun stream.
         * \param n est le nom de la catégorie
         * \param loglevel est le niveau de priorité minimal
         */
        static void InitNull(const std::string n = "default", arp_core::log::Priority::Value loglevel = arp_core::log::INFO);

        /** InitConsole initialise le Logger pour qu'il publie dans la sortie standard de la console.
         * \param n est le nom de la catégorie
         * \param loglevel est le niveau de priorité minimal
         */
        static void InitConsole(const std::string n = "default", arp_core::log::Priority::Value loglevel = arp_core::log::INFO);

        /** InitFile initialise le Logger pour qu'il publie dans un fichier.
         * \param n est le nom de la catégorie
         * \param loglevel est le niveau de priorité minimal
         * \remark Le nom du fichier est le nom de la catégorie adjoint du suffixe ".log"
         */
        static void InitFile(const std::string n = "default", arp_core::log::Priority::Value loglevel = arp_core::log::INFO);

        /**
         * Permet de changer le nom de la catégorie en cours de route.
         * \warning Ne change pas le nom du fichier dans le cas d'une publication dans un fichier.
         */
        static void SetName(const std::string n);

        /**
         * Permet de changer le niveau de priorité
         */
        static void SetLogLevel(const arp_core::log::Priority::Value loglevel);

        /**
         * Permet d'obtenir le CategoryStream associé au niveau de priorité spécifié.
         */
        static arp_core::log::CategoryStream getStream(arp_core::log::Priority::Value priority);

        /**
         * Permet d'accéder à la Category.
         */
        static arp_core::log::Category& Category();

    private:
        DefaultLogger(){};  // Private so that it can  not be called
        DefaultLogger(DefaultLogger const&){};             // copy constructor is private
        DefaultLogger& operator=(DefaultLogger const&){ return *this; };  // assignment operator is private

    private:
        static arp_core::log::Logger* m_pInstance;
        static arp_core::log::Category* m_pCat;
};

/*! @} End of Doxygen Groups*/

}
}

#endif //_ARP_RLU_KFL_LOGGER_HPP_
