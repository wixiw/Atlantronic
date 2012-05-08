/*
 * Logger.cpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#include "DefaultLogger.hpp"

#include <iostream>
#include <ostream>

using namespace std;

namespace arp_core { namespace log {


// Global static pointer used to ensure a single instance of the class.
arp_core::log::Logger* DefaultLogger::m_pInstance = NULL;
arp_core::log::Category* DefaultLogger::m_pCat = NULL;

void DefaultLogger::SetName(const std::string  n)
{
  if (!m_pInstance)
  {
      InitNull();
  }
  m_pCat->setName(n);
}

void DefaultLogger::SetLogLevel(const arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
      InitNull();
  }
  m_pCat->setLogLevel(loglevel);
}

arp_core::log::CategoryStream DefaultLogger::getStream(arp_core::log::Priority::Value priority)
{
  if (!m_pInstance)
  {
      InitNull();
  }
  return m_pCat->getStream(priority);
}

void DefaultLogger::InitConsole(const std::string n, arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
    m_pInstance = new arp_core::log::SimpleLogger();
    m_pCat = m_pInstance->create(n);
    SetLogLevel(loglevel);
  }
}

void DefaultLogger::InitFile(const std::string n, arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
    m_pInstance = new arp_core::log::FileLogger();
    m_pCat = m_pInstance->create(n);
    SetLogLevel(loglevel);
  }
}

void DefaultLogger::InitNull(const std::string n, arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
    m_pInstance = new arp_core::log::NullLogger();
    m_pCat = m_pInstance->create(n);
    SetLogLevel(loglevel);
  }
}

arp_core::log::Category& DefaultLogger::Category()
{
  if (!m_pInstance)
  {
      InitNull();
  }

  return *m_pCat;
}


arp_core::log::CategoryStream Log(arp_core::log::Priority::Value priority)
{
    return arp_core::log::DefaultLogger::Category() << priority;
}


}}
