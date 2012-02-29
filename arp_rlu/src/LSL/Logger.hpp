/*
 * Logger.hpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_LOGGER_HPP_
#define _ARP_RLU_LSL_LOGGER_HPP_

#include <logger/Logger.hpp>

namespace arp_rlu
{

namespace lsl
{

arp_core::log::CategoryStream Log(arp_core::log::Priority::Value priority);

class Logger
{
public:
  static void InitNull(const std::string n = "LSL", arp_core::log::Priority::Value loglevel = arp_core::log::INFO);
  static void InitConsole(const std::string n = "LSL", arp_core::log::Priority::Value loglevel = arp_core::log::INFO);
  static void InitFile(const std::string n = "LSL", arp_core::log::Priority::Value loglevel = arp_core::log::DEBUG);
  static void SetName(const std::string n);
  static void SetLogLevel(const arp_core::log::Priority::Value loglevel);
  static arp_core::log::CategoryStream getStream(arp_core::log::Priority::Value priority);
  static arp_core::log::Category& Category();

private:
  Logger(){};  // Private so that it can  not be called
  Logger(Logger const&){};             // copy constructor is private
  Logger& operator=(Logger const&){};  // assignment operator is private

private:
  static arp_core::log::Logger* m_pInstance;
  static arp_core::log::Category* m_pCat;
};

}
}

#endif //_ARP_RLU_LSL_LOGGER_HPP_
