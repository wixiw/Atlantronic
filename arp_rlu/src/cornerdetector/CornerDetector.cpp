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

CornerDetector::CornerDetector()
{
}

CornerDetector::~CornerDetector()
{
}

void CornerDetector::setScan(const Scan & scan)
{
    mScan = scan;
}

void CornerDetector::compute()
{
    /*split();
     merge();*/
    // and then compute pose
}

btTransform CornerDetector::getPose() const
{
    return btTransform();
}

std::vector<Segment> CornerDetector::split(const Scan & scan, double threshDist, unsigned int threshNumber)
{
    unsigned int n = scan.cols();

    if (n < threshNumber)
    {
        std::vector<Segment> result;
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

    if (n == 0)
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

    std::pair<Segment, SegmentedScan> p =  improveSegment(iSegmentedScan, std::make_pair(couples(fuseIndex,0),couples(fuseIndex,1)));

    unsigned int maxSgmtIndex = p.second.row(2).maxCoeff();
    if( maxSgmtIndex < sgmts.size() )
    {
        std::cerr << "CornerDetector merge : new sgmt index < vector<Segment> size => abnormal" << std::endl;
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

    Segment sgmt;
    sgmt.d = d;
    sgmt.alpha = alpha;
    sgmt.angleBegin = angleBegin;
    sgmt.angleEnd = angleEnd;
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
        std::cerr << "CornerDetector improveSegment : No corresponding indices in SegmentedScan" << std::endl;
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
