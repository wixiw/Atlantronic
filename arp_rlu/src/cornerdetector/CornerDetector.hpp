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
struct Corner
{
        double d1;
        double alpha1;
        double d2;
        double alpha2;
        double diag;
        double theta;
        double cornerAngle;
};

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
        Corner compute();
        btTransform getPose() const;
        void setSplitThresholdDistance(double val);
        void setSplitThresholdNumber(unsigned int val);
        void setMergeThresholdDistance(double val);
        void setMergeThresholdAngle(double val);



        std::vector<Segment> split(const Scan & s, double threshDist, unsigned int threshNumber);
        std::pair<std::vector<Segment>, SegmentedScan> merge(std::vector<Segment> sgmts,
                const SegmentedScan & iSegmentedScan, double threshDistance, double threshAngle);
        Segment computeSegment(Eigen::MatrixXd);
        std::pair<Segment, SegmentedScan> improveSegment(SegmentedScan scan, std::pair<unsigned int, unsigned int> fuseIndices);
        SegmentedScan attributeSegment(Scan scan, std::vector<Segment>);
        Corner extractCorner( std::vector<Segment> sgmts );

    protected:
        Scan mScan;
        btTransform mPose;
        double mSplitThreshDistance;
        unsigned int mSplitThreshNumber;
        double mMergeThreshDistance;
        double mMergeThreshAngle;

};

}

#endif /* _ARP_RLU_CORNERDETECTOR_HPP_ */
