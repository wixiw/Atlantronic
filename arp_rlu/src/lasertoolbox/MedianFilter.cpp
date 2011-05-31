/*
 * MedianFilter.cpp
 *
 *  Created on: 31 mai 2011
 *      Author: ard
 */

#include "MedianFilter.hpp"

using namespace arp_rlu;
using namespace Eigen;

#ifndef ROS_WARN
#define ROS_WARN(msg) (std::cout << msg << std::endl)
#endif

MedianFilter::MedianFilter(unsigned int w):
        width(w)
{
    rawScan = MatrixXd(0,0);
    filtScan = MatrixXd(0,0);
}

void MedianFilter::setWidth(unsigned int w)
{
    width = w;
}

void MedianFilter::setScan(Scan s)
{
    rawScan = s;
}

double MedianFilter::getMedian(Eigen::VectorXd raw)
{
    Eigen::VectorXd m = raw;
    unsigned int n = m.size();
    if( n == 0 )
        return 0.;

    if( n == 1 )
        return m(0);

    bool no_change;
    do
    {
        no_change = true;
        for(unsigned int j = 0; j < n-1 ; j++)
        {
            if(m(j) > m(j+1))
            {
                double tmp = m(j+1);
                m(j+1) = m(j);
                m(j) = tmp;
                no_change = false;
            }
        }
    }while(!no_change);

    return m((unsigned int)((n-1) / 2));
}

void MedianFilter::compute()
{

    if( rawScan.cols() == 0 )
    {
        ROS_WARN("MedianFilter compute : raw Scan is empty");
        filtScan = MatrixXd::Zero(0,0);
        return;
    }

    filtScan = MatrixXd(2, rawScan.cols());
    filtScan = rawScan;

    if( width == 0 )
    {
        ROS_WARN("MedianFilter compute : width filter is zero => filter do nothing");
        return;
    }

    int infIndex = (int)((width-1)/2);
    int supIndex = (int)(width-1-(width-1)/2);
    for(int j = 0; j < rawScan.cols() ; j++)
    {
        if( j-infIndex < 0 )
            continue;

        if( j+supIndex > rawScan.cols() - 1 )
            continue;

        VectorXd v(width);
        for(int k = 0; k < width ; k++)
            v(k) = rawScan(1,j-infIndex+k);
        filtScan(1,j) = getMedian(v);
    }
    return;
}

Scan MedianFilter::getResult()
{
    return filtScan;
}

