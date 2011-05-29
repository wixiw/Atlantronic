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
typedef Eigen::MatrixXd SegmentedScan;

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

        std::vector<Segment> split(const Scan & s, double threshDist, unsigned int threshNumber);
        std::pair<std::vector<Segment>, SegmentedScan> merge(std::vector<Segment> sgmts,
                const SegmentedScan & iSegmentedScan, double threshDistance, double threshAngle);
        Segment computeSegment(Eigen::MatrixXd);
        std::pair<Segment, SegmentedScan> improveSegment(SegmentedScan scan, std::pair<unsigned int, unsigned int> fuseIndices);

    protected:
        Scan mScan;
        btTransform mPose;

};

}

#endif /* _ARP_RLU_CORNERDETECTOR_HPP_ */
