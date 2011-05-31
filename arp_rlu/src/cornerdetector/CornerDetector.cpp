/*
 * CornerDetector.cpp
 *
 *  Created on: 28 mai 2011
 *      Author: Boris
 */

#include "CornerDetector.hpp"
#include <Eigen/SVD>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;

CornerDetector::CornerDetector() :
    mSplitThreshDistance(0.05), mSplitThreshNumber(6), mMergeThreshDistance(0.2), mMergeThreshAngle(15 * PI / 180)
{
}

CornerDetector::~CornerDetector()
{
}

void CornerDetector::setScan(const Scan & scan)
{
    mScan = scan;
}

Corner CornerDetector::compute()
{
    if(mScan.cols() < 10)
    {
        ROS_WARN("CornerDetector compute : Scan is empty");
        return Corner();
    }

    std::vector<Segment> sgmts = split(mScan, mSplitThreshDistance, mSplitThreshNumber);
    SegmentedScan sgmtScan = attributeSegment(mScan, sgmts);
    std::pair<std::vector<Segment>, SegmentedScan> p;
    p = merge(sgmts, sgmtScan, mMergeThreshDistance, mMergeThreshAngle);

    return extractCorner(sgmts);

}

btTransform CornerDetector::getPose() const
{
    return btTransform();
}

void CornerDetector::setSplitThresholdDistance(double val)
{
    this->mSplitThreshDistance = val;
}

void CornerDetector::setSplitThresholdNumber(unsigned int val)
{
    this->mSplitThreshNumber = val;
}

void CornerDetector::setMergeThresholdDistance(double val)
{
    this->mMergeThreshDistance = val;
}

void CornerDetector::setMergeThresholdAngle(double val)
{
    this->mMergeThreshAngle = val;
}

std::vector<Segment> CornerDetector::split(const Scan & scan, double threshDist, unsigned int threshNumber)
{
    unsigned int n = scan.cols();

    if (n < threshNumber)
    {
        std::vector<Segment> result;
        ROS_WARN("CornerDetector split : Not enough points in Scan");
        return result; // empty segments vector
    }

    Scan extremities(2, 2);
    extremities.col(0) = scan.leftCols(1);
    extremities.col(1) = scan.rightCols(1);
    Segment sgmt = computeSegment(extremities);
    double d = sgmt.d;
    double alpha = sgmt.alpha;

    VectorXd distances(n);
    for (unsigned int i = 0; i < n; i++)
    {
        distances(i) = abs(scan(1, i) * cos(alpha - scan(0, i)) - d);
    }
    int i_max;
    double d_max = distances.maxCoeff(&i_max);

    if (d_max < threshDist)
    {
        // We have a segment !
        Segment sgmt = computeSegment(scan);
        std::vector<Segment> result;
        result.push_back(sgmt);
        return result;
    }
    else
    {
        // We split in 2 pieces
        std::vector<Segment> left;
        left = split(scan.leftCols(i_max + 1), threshDist, threshNumber);
        std::vector<Segment> right;
        right = split(scan.rightCols(n - i_max - 1), threshDist, threshNumber);

        left.insert(left.end(), right.begin(), right.end());
        return left;
    }

    std::vector<Segment> result;
    return result;
}

std::pair<std::vector<Segment>, SegmentedScan> CornerDetector::merge(std::vector<Segment> sgmts,
        const SegmentedScan & iSegmentedScan, double threshDistance, double threshAngle)
{
    unsigned int n = sgmts.size();
    if (n < 2)
    {
        return std::make_pair(sgmts, iSegmentedScan);
    }

    MatrixXi couples = MatrixXi::Zero(n * (n - 1) / 2, 5);
    unsigned int k = 0;
    for (unsigned int i = 0; i < n - 1; i++)
    {
        for (unsigned int j = i + 1; j < n; j++)
        {
            couples(k, 0) = i;
            couples(k, 1) = j;
            k++;
        }
    }

    for (unsigned int i = 0; i < n * (n - 1) / 2; i++)
    {
        Segment sgmt1 = sgmts[couples(i, 0)];
        Segment sgmt2 = sgmts[couples(i, 1)];

        if (abs(sgmt1.d - sgmt2.d) < threshDistance)
        {
            couples(i, 2) = 1;
        }
        if (abs(normalizeAngle(sgmt1.alpha - sgmt2.alpha)) < threshAngle)
        {
            couples(i, 3) = 1;
        }
        couples(i, 4) = couples(i, 2) && couples(i, 3);
    }

    unsigned int nbFusion = couples.col(4).sum();

    if (nbFusion == 0)
    {
        return std::make_pair(sgmts, iSegmentedScan);
    }

    // Fuse first segment pair
    unsigned int fuseIndex = 0;
    for (unsigned int i = 0; i < n * (n - 1) / 2; i++)
    {
        if (couples(i, 4) > 0)
        {
            fuseIndex = i;
            break;
        }
    }

    std::pair<Segment, SegmentedScan> p = improveSegment(iSegmentedScan,
            std::make_pair(couples(fuseIndex, 0), couples(fuseIndex, 1)));

    unsigned int maxSgmtIndex = p.second.row(2).maxCoeff();
    if (maxSgmtIndex < sgmts.size())
    {
        ROS_WARN("CornerDetector merge : new sgmt index < vector<Segment> size => abnormal");
    }
    std::vector<Segment> newSgmts = sgmts;
    newSgmts.push_back(p.first);

    return std::make_pair(newSgmts, p.second);

}

Segment CornerDetector::computeSegment(Eigen::MatrixXd scan)
{
    unsigned int n = scan.cols();

    MatrixXd cartesianScan(2, n);
    for (unsigned int i = 0; i < n; i++)
    {
        cartesianScan(0, i) = scan(1, i) * cos(scan(0, i));
        cartesianScan(1, i) = scan(1, i) * sin(scan(0, i));
    }

    MatrixXd A(n, 3);
    A.block(0, 0, n, 1) = cartesianScan.block(0, 0, 1, n).transpose();
    A.block(0, 1, n, 1) = cartesianScan.block(1, 0, 1, n).transpose();
    A.block(0, 2, n, 1) = -MatrixXd::Ones(n, 1);
    JacobiSVD < MatrixXd > svd(A, ComputeFullU | ComputeFullV);
    MatrixXd s = svd.singularValues();
    MatrixXd v = svd.matrixV();
    VectorXd u = v.block(0, 2, 3, 1);

    double alpha = atan2(u(1), u(0));
    double d = u(2) / u.block(0, 0, 2, 1).norm();

    if (d < 0.)
    {
        d = -d;
        alpha = alpha + PI;
    }
    alpha = fmod(alpha + 4. * PI, 2. * PI);
    double angleBegin = scan(0, 0);
    double angleEnd = scan(0, n - 1);
    double length = sqrt( (cartesianScan(0, n-1) - cartesianScan(0, 0)) * (cartesianScan(0, n-1) - cartesianScan(0, 0))
            + (cartesianScan(1, n-1) - cartesianScan(1, 0)) * (cartesianScan(1, n-1) - cartesianScan(1, 0)));

    Segment sgmt;
    sgmt.d = d;
    sgmt.alpha = alpha;
    sgmt.angleBegin = angleBegin;
    sgmt.angleEnd = angleEnd;
    sgmt.length = length;
    sgmt.nbMeas = n;
    return sgmt;
}

std::pair<Segment, SegmentedScan> CornerDetector::improveSegment(SegmentedScan scan,
        std::pair<unsigned int, unsigned int> indices)
{
    unsigned int nbPoints = 0;
    for (unsigned int i = 0; i < scan.cols(); i++)
    {
        if (scan(2, i) == indices.first || scan(2, i) == indices.second)
        {
            nbPoints++;
        }
    }
    if (nbPoints == 0)
    {
        ROS_WARN("CornerDetector improveSegment : No corresponding indices in SegmentedScan");
        return std::make_pair(Segment(), scan);
    }

    Scan selectedMeas(2, nbPoints);
    unsigned int index = 0;
    for (unsigned int i = 0; i < scan.cols(); i++)
    {
        if (scan(2, i) == indices.first || scan(2, i) == indices.second)
        {
            selectedMeas(0, index) = scan(0, i);
            selectedMeas(1, index) = scan(1, i);
            index++;
        }
    }

    Segment fuseSegment = computeSegment(selectedMeas);
    unsigned int maxSgmtIndice = scan.row(2).maxCoeff();
    SegmentedScan newSegScan = scan;
    for (unsigned int i = 0; i < scan.cols(); i++)
    {
        if (scan(2, i) == indices.first || scan(2, i) == indices.second)
        {
            newSegScan(2, i) = maxSgmtIndice + 1;
        }
    }

    return std::make_pair(fuseSegment, newSegScan);
}

SegmentedScan CornerDetector::attributeSegment(Scan scan, std::vector<Segment> sgmts)
{
    SegmentedScan sgmScan = -MatrixXd::Ones(3, scan.cols());
    sgmScan.topRows(2) = scan;
    for (unsigned int i = 0; i < sgmts.size(); i++)
    {
        for (unsigned int k = 0; k < scan.cols(); k++)
        {
            if (scan(0, k) > sgmts[i].angleBegin)
            {
                if (scan(0, k) > sgmts[i].angleEnd)
                {
                    break;
                }
                sgmScan(2, k) = i;
            }
        }
    }
    return sgmScan;
}

Corner CornerDetector::extractCorner(std::vector<Segment> sgmts)
{
    unsigned int n = sgmts.size();
    ROS_INFO("*******************************");
    ROS_INFO("Nb of segments : %d", n);
    for (unsigned int i = 0; i < n; i++)
    {
        ROS_INFO("Segment %d:", i);
        ROS_INFO("  d = %f", sgmts[i].d);
        ROS_INFO("  alpha = %f", sgmts[i].alpha);
        ROS_INFO("  length = %f", sgmts[i].length);
        ROS_INFO("  angleBegin = %f", sgmts[i].angleBegin);
        ROS_INFO("  angleEnd = %f", sgmts[i].angleEnd);
    }

    if (n < 2)
    {
        ROS_WARN("CornerDetector extractCorner : No corner found because less than 2 segments detected");
        return Corner();
    }

    MatrixXd corners = MatrixXd::Zero(n * (n - 1) / 2, 3);
    unsigned int k = 0;
    for (unsigned int i = 0; i < n - 1; i++)
    {
        for (unsigned int j = i + 1; j < n; j++)
        {
            corners(k, 0) = i;
            corners(k, 1) = j;
            corners(k, 2) = abs(normalizeAngle(sgmts[j].alpha - sgmts[i].alpha) - PI / 2.);
            k++;
        }
    }

    ROS_INFO_STREAM("Corners Matrix : " << std::endl << corners);

    int i_min;
    double alphaMin = corners.col(2).minCoeff(&i_min);
    ROS_INFO("i_min = %d with alphaMin = %f", i_min, alphaMin);

    double cornerAngle = PI - abs(normalizeAngle(sgmts[corners(i_min, 0)].alpha - sgmts[corners(i_min, 1)].alpha));
    ROS_INFO("cornerAngle (rad) = %f", cornerAngle);

    if( cornerAngle < PI/2. - 10. * PI/ 180. || cornerAngle > PI/2. + 10. * PI/ 180.)
    {
        ROS_WARN("Detected corner is eliminated because its angle is not near 90 deg (%f)", rad2deg(cornerAngle));
        return Corner();
    }

    Corner c;
    c.d1 = sgmts[corners(i_min, 0)].d;
    c.alpha1 = sgmts[corners(i_min, 0)].alpha;
    c.length1 = sgmts[corners(i_min, 0)].length;
    c.angleBegin1 = sgmts[corners(i_min, 0)].angleBegin;
    c.angleEnd1 = sgmts[corners(i_min, 0)].angleEnd;
    c.d2 = sgmts[corners(i_min, 1)].d;
    c.alpha2 = sgmts[corners(i_min, 1)].alpha;
    c.length2 = sgmts[corners(i_min, 1)].length;
    c.angleBegin2 = sgmts[corners(i_min, 1)].angleBegin;
    c.angleEnd2 = sgmts[corners(i_min, 1)].angleEnd;
    c.diag = sqrt(c.d1 * c.d1 + c.d2 * c.d2);
    c.theta = fmod(c.alpha1 + atan2(c.d2, c.d1) + 2. * PI, 2 * PI);
    c.cornerAngle = cornerAngle;

    return c;
}

