#include <rtt/types/StructTypeInfo.hpp>
#include "math/EstimatedTwist2D.hpp"
#include "math/MathFactory.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

#ifndef ESTIMATEDTWIST2DTYPEINFO_HPP_
#define ESTIMATEDTWIST2DTYPEINFO_HPP_

using namespace arp_math;
using namespace RTT;

namespace boost {
namespace serialization {
// The helper function which you write yourself:
template<class Archive>
void serialize( Archive & a, EstimatedTwist2D& twist, unsigned int) {
        using boost::serialization::make_nvp;
        a & make_nvp("vx", twist.vxRef());
        a & make_nvp("vy", twist.vyRef());
        a & make_nvp("vtheta", twist.vhRef());
        //TODO à remettre quand un typekit long double sera dispo
        //a & make_nvp("date", twist.dateRef());
}
}
}


//  // Displaying:
//  std::ostream& operator<<(std::ostream& os, const EstimatedTwist2D& pose)
//  {
//      return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
//  }
//
//  // Reading :
//  std::istream& operator>>(std::istream& is, EstimatedTwist2D& cd) {
//      char c;
//     return is >> c >> cd;
//  }

EstimatedTwist2D createEstimatedTwist2D(double vx, double vy, double vangle)
{
    return MathFactory::createEstimatedTwist2DFromCartesianRepr(vx,vy,vangle, 0., Covariance3::Identity());
}

// The RTT helper class which uses the above function behind the scenes:
struct EstimatedTwist2DTypeInfo: public RTT::types::StructTypeInfo<EstimatedTwist2D,false>
{
    EstimatedTwist2DTypeInfo():
        RTT::types::StructTypeInfo<EstimatedTwist2D,false>("EstimatedTwist2D")
        {

        }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<EstimatedTwist2D>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<EstimatedTwist2D> >( in );
        data->reset();
        data->evaluate();
        EstimatedTwist2D t = data->get();
        os << " vx: " << t.vx() << " m/s +- " << 1.5 * sqrt(t.cov()(0,0)) * 1000. << " mm/s ,";
        os << " vy: " << t.vy() << " m/s +- " << 1.5 * sqrt(t.cov()(1,1)) * 1000. << " mm/s ,";
        os << " vh:" << rad2deg(t.vh()) << " deg/s +- " << 1.5 * rad2deg(sqrt(t.cov()(1,1))) << " deg/s ,";
        os << " date = " << t.date() << " sec";
        return os;
    }

    virtual std::istream& read(std::istream& os, base::DataSourceBase::shared_ptr out ) const
    {
        return os;
    }
};

#endif
