/*
 * DetectedCircle.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "DetectedCircle.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

DetectedCircle::DetectedCircle()
 : Circle()
 , DetectedObject()
{
}

DetectedCircle::DetectedCircle(const DetectedObject & ls)
 : Circle()
 , DetectedObject(ls)
{
}
