/*
 * MedianFilter.hpp
 *
 *  Created on: 31 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_MEDIANFILTER_HPP_
#define _ARP_RLU_MEDIANFILTER_HPP_

#include "lasertoolbox/common.hpp"

namespace arp_rlu
{
    class MedianFilter
    {
        public:
            MedianFilter(unsigned int w = 3);

            void setWidth(unsigned int);
            void setScan(Scan);
            void compute();
            Scan getResult();

            double getMedian(Eigen::VectorXd m);

        protected:
            unsigned int width;
            Scan rawScan;
            Scan filtScan;

    };
}

#endif /* _ARP_RLU_MEDIANFILTER_HPP_ */
