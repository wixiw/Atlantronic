/*
 * LaserScan.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_LASERSCAN_HPP_
#define _ARP_RLU_LSL_LASERSCAN_HPP_

#include <math/core>

namespace arp_rlu
{

namespace lsl
{

class LaserScan
{
    public:
        LaserScan();
        ~LaserScan();

        unsigned int getSize() const;
        bool computeCartesianData(Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh);
        void setPolarData(Eigen::MatrixXd data);
        Eigen::MatrixXd getPolarData() const;
        Eigen::MatrixXd getCartesianData() const;
        bool areCartesianDataAvailaible();
        bool cleanUp();

    protected:

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_LASERSCAN_HPP_ */
