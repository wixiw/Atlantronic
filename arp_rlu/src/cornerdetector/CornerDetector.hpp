/*
 * CornerDetector.hpp
 *
 *  Created on: 28 mai 2011
 *      Author: Boris
 */

#ifndef _ARP_RLU_CORNERDETECTOR_HPP_
#define _ARP_RLU_CORNERDETECTOR_HPP_

#include "tf/tf.h"
#include <math/math.hpp>

#include <vector>

namespace arp_rlu
{

typedef Eigen::MatrixXd Scan;

struct Segment
{
        double d;
        double alpha;
        double angleBegin;
        double angleEnd;
        int nbMeas;
};

class CornerDetector
{
    public:
        CornerDetector();
        ~CornerDetector();

        void setScan(const Scan &);
        void compute();
        btTransform getPose() const;

        std::vector<Segment> split(const Scan & s, double threshDist, int threshNumber);
        void merge();
        Segment computeSegment( Eigen::MatrixXd );
        Segment improveSegment( Eigen::MatrixXd );

    protected:
        Scan mScan;
        btTransform mPose;

};

}

#endif /* _ARP_RLU_CORNERDETECTOR_HPP_ */
