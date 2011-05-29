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

std::vector<Segment> CornerDetector::split(const Scan & scan, double threshDist, int threshNumber)
{
    unsigned int n = scan.cols();

    if (n < threshNumber)
    {
        std::vector<Segment> result;
        return result;  // empty segments vector
    }

    Scan extremities(2,2);
    extremities.col(0) = scan.leftCols(1);
    extremities.col(1) = scan.rightCols(1);
    Segment sgmt = computeSegment(extremities);
    double d = sgmt.d;
    double alpha = sgmt.alpha;

    VectorXd distances(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        distances(i) = abs(scan(1,i) * cos(alpha - scan(0,i)) - d);
    }
    int i_max;
    double d_max = distances.maxCoeff(&i_max);

    if( d_max < threshDist )
    {
        // We have a segment !
        Segment sgmt = computeSegment( scan );
        std::vector<Segment> result;
        result.push_back( sgmt );
        return result;
    }
    else
    {
        // We split in 2 pieces
        std::vector<Segment> left;
        left = split( scan.leftCols(i_max + 1 ), threshDist, threshNumber );
        std::vector<Segment> right;
        right = split( scan.rightCols( n - i_max - 1 ), threshDist, threshNumber );

        left.insert( left.end(), right.begin(), right.end() ) ;
        return left;
    }

    std::vector<Segment> result;
    return result;
}

void CornerDetector::merge()
{
}

Segment CornerDetector::computeSegment( Eigen::MatrixXd scan)
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
    double angleBegin = scan(0,0);
    double angleEnd = scan(0,n-1);

    Segment sgmt;
    sgmt.d = d;
    sgmt.alpha = alpha;
    sgmt.angleBegin = angleBegin;
    sgmt.angleEnd = angleEnd;
    sgmt.nbMeas = n;
    return sgmt;
}

Segment improveSegment(Scan)
{
    return Segment();
}
