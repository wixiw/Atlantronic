/*
 * uTest_LSL.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE uTest_LSL
#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>

#include "uTest_LaserScan.hpp"

#include "objects/uTest_DetectedObject.hpp"
#include "objects/uTest_Circle.hpp"
#include "objects/uTest_DetectedCircle.hpp"

#include "filters/uTest_MedianFilter.hpp"
#include "filters/uTest_PolarCrop.hpp"
#include "filters/uTest_CartesianCrop.hpp"
#include "filters/uTest_PolarSegment.hpp"
#include "filters/uTest_CartesianSegment.hpp"
//#include "filters/uTest_CircleIdentif.hpp"

