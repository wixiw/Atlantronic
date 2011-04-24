/*
 * BiteTest.hpp
 *
 *  Created on: 26 oct. 2010
 *      Author: ard
 */

#ifndef BITETEST_HPP_
#define BITETEST_HPP_

#include <taskcontexts/ARDTest.hpp>

using namespace arp_core;

namespace CoreTest
{

    class BiteTest: public CoreTest::ARDTest
    {
    public:
        BiteTest(const std::string& name);
    };

}

#endif /* BITETEST_HPP_ */
