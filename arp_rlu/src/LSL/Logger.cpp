/*
 * Logger.cpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#include "LSL/Logger.hpp"

#include <iostream>
#include <ostream>

using namespace arp_rlu;
using namespace std;
using namespace lsl;

namespace arp_rlu { namespace lsl {


// Global static pointer used to ensure a single instance of the class.
arp_core::log::Logger* Logger::m_pInstance = NULL;
arp_core::log::Category* Logger::m_pCat = NULL;

void Logger::SetName(const std::string  n)
{
  if (!m_pInstance)
  {
      InitNull();
  }
  m_pCat->setName(n);
}

void Logger::SetLogLevel(const arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
      InitNull();
  }
  m_pCat->setLogLevel(loglevel);
}

arp_core::log::CategoryStream Logger::getStream(arp_core::log::Priority::Value priority)
{
  if (!m_pInstance)
  {
      InitNull();
  }
  return m_pCat->getStream(priority);
}

void Logger::InitConsole(const std::string n, arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
    m_pInstance = new arp_core::log::SimpleLogger();
    m_pCat = m_pInstance->create(n);
    SetLogLevel(loglevel);
  }
}

void Logger::InitFile(const std::string n, arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
    m_pInstance = new arp_core::log::FileLogger();
    m_pCat = m_pInstance->create(n);
    SetLogLevel(loglevel);
  }
}

void Logger::InitNull(const std::string n, arp_core::log::Priority::Value loglevel)
{
  if (!m_pInstance)
  {
    m_pInstance = new arp_core::log::NullLogger();
    m_pCat = m_pInstance->create(n);
    SetLogLevel(loglevel);
  }
}

arp_core::log::Category& Logger::Category()
{
  if (!m_pInstance)
  {
      InitNull();
  }

  return *m_pCat;
}


arp_core::log::CategoryStream Log(arp_core::log::Priority::Value priority)
{
    return arp_rlu::lsl::Logger::Category() << priority;
}


}}
