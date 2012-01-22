/*
 * KFLocalizator.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "KFLocalizator.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace kfl;

KFLocalizator::Params::Params()
{
}

std::string KFLocalizator::Params::getInfo()
{
    std::stringstream ss;
    ss << "KFLocalizator params :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

KFLocalizator::InitParams::InitParams()
{
}

std::string KFLocalizator::InitParams::getInfo()
{
    std::stringstream ss;
    ss << "KFLocalizator InitParams :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

KFLocalizator::IEKFParams::IEKFParams()
{
}

std::string KFLocalizator::IEKFParams::getInfo()
{
    std::stringstream ss;
    ss << "KFLocalizator IEKFParams :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

KFLocalizator::KFLocalizator()
{
}

KFLocalizator::~KFLocalizator()
{
}
