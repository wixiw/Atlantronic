/*
 * LocalizatorTypes.hpp
 *
 *  Created on: Mar 22, 2014
 *      Author: willy
 */

#ifndef LOCALIZATORTYPES_HPP_
#define LOCALIZATORTYPES_HPP_

namespace arp_rlu
{

enum LocalizationState
{
    STOPPED = 0, RUNNING = 1
};

enum LocalizationMode
{
    ODO_ONLY = 0, SMOOTH = 1, FUSION = 2
};

enum LocalizationQuality
{
    LOST = 0, BAD = 1, GOOD = 2
};

enum LocalizationVisibility
{
    NONE = 0, OCCULTED = 1, SEGMENT = 2, TRIANGLE = 3,
};

extern const char* const LocalizationStateNames[];
extern const char* const LocalizationModeNames[];
extern const char* const LocalizationQualityNames[];
extern const char* const LocalizationVisibilityNames[];

}

#endif /* LOCALIZATORTYPES_HPP_ */
