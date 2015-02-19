/*
 * VersionMessage.hpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#ifndef VERSIONMESSAGE_HPP_
#define VERSIONMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"

#ifndef VERSION
#error VERSION not defined
#endif

namespace arp_stm32
{

/**
 * This class hold the stm32 version string message.
 * This version is compared to the pre-compiled one.
 * Both should be equal are compatibility issues are possible.
 */
class VersionMessage: public IpcMsg
{
    public:
        static const MsgSize SIZE = 41;
        static const char EXPECTED_VERSION[SIZE];

        VersionMessage();
        virtual ~VersionMessage(){};

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
         * getter
         */
        char const* getVersion() const;

        /**
         * setter
         */
        void setVersion(char const * const version);

    protected:
        char m_version[SIZE];
};

} /* namespace arp_stm32 */
#endif /* VERSIONMESSAGE_HPP_ */
