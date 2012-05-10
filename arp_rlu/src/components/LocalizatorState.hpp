/*
 * LocalizatorState.hpp
 *
 *  Created on: 10 Mai 2012
 *      Author: ard
 */

#ifndef LOCALIZATORSTATE_HPP_
#define LOCALIZATORSTATE_HPP_

namespace arp_rlu
{
enum LocalizationState
{
    __STOPED__ = 0,
    _ODO_ONLY_ = 1,
    __FUSION__ = 2,
    _BAD__ODO_ = 3,
    ___LOST___ = 4
};
}

#endif
