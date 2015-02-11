/*
 * ConfigurationMsg.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef ConfigurationMsg_HPP_
#define ConfigurationMsg_HPP_

#include "ipc/IpcMsg.hpp"
#include "discovery/stm32_config.h"

namespace arp_stm32
{

class ConfigurationMsg: public arp_stm32::IpcMsg
{
    public:
        ConfigurationMsg();
        ConfigurationMsg(stm32_config const& config);
        virtual ~ConfigurationMsg();

        static const MsgSize SIZE = sizeof(stm32_config);

        /**
         * Overloaded \see IpcMsg
         * convert the string version
         */
        virtual bool serialize(Payload& payload) const;

        /**
         * Overloaded \see IpcMsg
         * ensure size is 41
         */
        virtual bool deserialize(PayloadConst payload);

        /**
         * Overloaded \see IpcMsg
         */
        virtual MsgType getType() const;

        /**
         * Overloaded \see IpcMsg
         */
        virtual MsgSize getSize() const;

        /**
         * Accessor
         * time of match in ms
         */
        uint32_t getMatchDuration() const;

    protected:
        stm32_config m_config;
};

} /* namespace arp_stm32 */
#endif /* ConfigurationMsg_HPP_ */
