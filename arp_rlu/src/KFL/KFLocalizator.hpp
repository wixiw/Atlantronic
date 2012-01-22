/*
 * KFLocalizator.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_KFLOCALIZATOR_HPP_
#define _ARP_RLU_KFL_KFLOCALIZATOR_HPP_

#include <math/math.hpp>

namespace arp_rlu
{

namespace kfl
{

class KFLocalizator
{
    class InitParams
    {
        InitParams();
        std::string getInfo();
    };

    class IEKFParams
    {
        IEKFParams();
        std::string getInfo();
    };

    class Params
    {
        Params();
        std::string getInfo();
    };

    public:
        KFLocalizator();
        ~KFLocalizator();


    protected:

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLOCALIZATOR_HPP_ */
