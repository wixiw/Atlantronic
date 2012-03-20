/*
 * Interpolator.cpp
 *
 *  Created on: 23 February 2012
 *      Author: Boris
 */

#include "Interpolator.hpp"
#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace Eigen;

Eigen::VectorXd Interpolator::transInterp(Eigen::VectorXd  tt,
        Eigen::VectorXd ttc,
        Eigen::VectorXd yyc)
{
    // ttc & yyc have different sizes
    if(ttc.size() != yyc.size())
    {
        return Eigen::VectorXd(0);
    }

    // N = 0
    if(ttc.size() == 0)
    {
        return Eigen::VectorXd(0);
    }

    // N = 1
    if(ttc.size() == 1)
    {
        return Eigen::VectorXd::Constant(tt.size(), yyc[0]);
    }

    // P = 0
    if(tt.size() == 0)
    {
        return Eigen::VectorXd(0);
    }

    // N > 1 & P > 0
    Eigen::VectorXd ret = Eigen::VectorXd(tt.size());
    for(unsigned int i = 0 ; i<tt.size() ; i++)
    {
        int index = find(tt[i], ttc);
        if(index < 0)
        {
            ret[i] = yyc[0] - (ttc[0]-tt[i])*(yyc[1]-yyc[0])/(ttc[1]-ttc[0]);
        }
        else if(index == ttc.size()-1)
        {
            ret[i] = yyc[index] + (tt[i]-ttc[index])*(yyc[index]-yyc[index-1])/(ttc[index]-ttc[index-1]);
        }
        else
        {
            ret[i] = yyc[index] + (tt[i]-ttc[index])*(yyc[index+1]-yyc[index])/(ttc[index+1]-ttc[index]);
        }
    }
    return ret;
}

Eigen::VectorXd Interpolator::rotInterp(Eigen::VectorXd tt,
        Eigen::VectorXd ttc,
        Eigen::VectorXd hhc)
{
    // ttc & yyc have different sizes
    if(ttc.size() != hhc.size())
    {
        return Eigen::VectorXd(0);
    }

    // N = 0
    if(ttc.size() == 0)
    {
        return Eigen::VectorXd(0);
    }

    // N = 1
    if(ttc.size() == 1)
    {
        return Eigen::VectorXd::Constant(tt.size(), betweenMinusPiAndPlusPi(hhc[0]));
    }

    // P = 0
    if(tt.size() == 0)
    {
        return Eigen::VectorXd(0);
    }

    // N > 1 & P > 0
    Eigen::VectorXd ret = Eigen::VectorXd(tt.size());
    for(unsigned int i = 0 ; i<tt.size() ; i++)
    {
        int index = find(tt[i], ttc);
        if(index < 0)
        {
            ret[i] = hhc[0] - (ttc[0]-tt[i])*betweenMinusPiAndPlusPi(hhc[1]-hhc[0])/(ttc[1]-ttc[0]);
        }
        else if(index == ttc.size()-1)
        {
            ret[i] = hhc[index] + (tt[i]-ttc[index])*betweenMinusPiAndPlusPi(hhc[index]-hhc[index-1])/(ttc[index]-ttc[index-1]);
        }
        else
        {
            ret[i] = hhc[index] + (tt[i]-ttc[index])*betweenMinusPiAndPlusPi(hhc[index+1]-hhc[index])/(ttc[index+1]-ttc[index]);
        }
        ret[i] = betweenMinusPiAndPlusPi(ret[i]);
    }
    return ret;
}

Eigen::Array< Eigen::Matrix3d, Dynamic, 1 > Interpolator::covInterp(Eigen::VectorXd  tt,
        Eigen::VectorXd ttc,
        Eigen::Array< Eigen::Matrix3d, Dynamic, 1 > covc,
        double epsilon,
        Eigen::Vector3d minimum)
{
    // ttc & yyc have different sizes
    if(ttc.size() != covc.size())
    {
        return Eigen::Array< Eigen::Matrix3d, Dynamic, 1 >(0);
    }

    // N = 0
    if(ttc.size() == 0)
    {
        return Eigen::Array< Eigen::Matrix3d, Dynamic, 1 >(0);
    }

    // N = 1
    if(ttc.size() == 1)
    {
        Eigen::Array< Eigen::Matrix3d, Dynamic, 1 > ret(tt.size());
        for(unsigned int k = 0 ; k<tt.size() ; k++)
        {
            ret(k) = covc(0);
        }
        return ret;
    }

    // P = 0
    if(tt.size() == 0)
    {
        return Eigen::Array< Eigen::Matrix3d, Dynamic, 1 >(0);
    }

    // N > 1 & P > 0
    Eigen::Array< Eigen::Matrix3d, Dynamic, 1 > ret(tt.size());
    for(unsigned int k = 0 ; k<tt.size() ; k++)
    {
        int index = find(tt[k], ttc);
        if(index < 0)
        {
            ret[k] = covc[0].array() - (ttc[0]-tt[k])*(covc[1].array()-covc[0].array())/(ttc[1]-ttc[0]);
        }
        else if(index == ttc.size()-1)
        {
            ret[k] = covc[index].array() + (tt[k]-ttc[index])*(covc[index].array()-covc[index-1].array())/(ttc[index]-ttc[index-1]);
        }
        else
        {
            ret[k] = covc[index].array() + (tt[k]-ttc[index])*(covc[index+1].array()-covc[index].array())/(ttc[index+1]-ttc[index]);
        }
        for(unsigned int i = 0 ; i < 3 ; i++)
        {
            for(unsigned int j = i ; j < 3 ; j++)
            {
                if(i == j)
                {
                    if(ret[k](j,i) < epsilon)
                    {
                        ret[k](j,i) = exp(ret[k](j,i) / (epsilon-minimum(i)) + log(epsilon-minimum(i)) - (epsilon/(epsilon-minimum(i)))) + minimum(i);
                    }
                }
                else
                {
                    double m = (ret[k](i,j) + ret[k](j,i)) / 2.;
//                    ret[k](i,j) = m > 0. ? m : 0.;
                    ret[k](i,j) = m;
                    ret[k](j,i) = ret[k](i,j);
                }
            }
        }
    }
    return ret;
}

int Interpolator::find(double t, Eigen::VectorXd ttc)
{
    // N = 0
    if(ttc.size() == 0)
    {
        return -1;
    }

    // N = 1
    if(ttc.size() == 1)
    {
        if(t < ttc[0])
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }

    // N > 1
    if(t < ttc[0])
    {
        return -1;
    }
    for(unsigned int i = 0 ; i < ttc.size()-1 ; i++)
    {
        if(ttc[i] <= t && t < ttc[i+1] )
        {
            return i;
        }
    }
    return ttc.size()-1;
}
