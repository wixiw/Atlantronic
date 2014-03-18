/*
 * Logger.hpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#ifndef _ARP_ODS_ORDER_LOGGER_HPP_
#define _ARP_ODS_ORDER_LOGGER_HPP_

#include <logger/Logger.hpp>

namespace arp_ods
{

/*!
 *  \addtogroup kfl
 *  @{
 */

/**
 *  \function Log
 *  Permet de logger avec le niveau de priorité spécifié.\n
 *  Exemple : Log( INFO ) << "exemple de message d'information";
 */
arp_core::log::CategoryStream Log(arp_core::log::Priority::Value priority);

/** \class Logger
 *
 * \brief Logger est la classe qui gère les log de KFL. C'est un singleton.
 *
 * Il s'initialise simplement en faisant par exemple :\n
 * arp_rlu::kfl::Logger::InitConsole("CategoryName", DEBUG);\n
 * ou :\n
 * arp_rlu::kfl::Logger::InitFile("FileName", DEBUG);
 *
 */
class Logger
{
    public:
        /** InitNull initialise le Logger pour qu'il publie dans aucun stream.
         * \param n est le nom de la catégorie
         * \param loglevel est le niveau de priorité minimal
         */
        static void InitNull(const std::string n = "ODS_orders", arp_core::log::Priority::Value loglevel = arp_core::log::INFO);

        /** InitConsole initialise le Logger pour qu'il publie dans la sortie standard de la console.
         * \param n est le nom de la catégorie
         * \param loglevel est le niveau de priorité minimal
         */
        static void InitConsole(const std::string n = "ODS_orders", arp_core::log::Priority::Value loglevel = arp_core::log::INFO);

        /** InitFile initialise le Logger pour qu'il publie dans un fichier.
         * \param n est le nom de la catégorie
         * \param loglevel est le niveau de priorité minimal
         * \remark Le nom du fichier est le nom de la catégorie adjoint du suffixe ".log"
         */
        static void InitFile(const std::string n = "ODS_orders", arp_core::log::Priority::Value loglevel = arp_core::log::DEBUG);

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
        Logger(){};  // Private so that it can  not be called
        Logger(Logger const&){};             // copy constructor is private
        Logger& operator=(Logger const&){ return *this; };  // assignment operator is private

    private:
        static arp_core::log::Logger* m_pInstance;
        static arp_core::log::Category* m_pCat;
};

/*! @} End of Doxygen Groups*/

}

#endif //_ARP_RLU_KFL_LOGGER_HPP_
