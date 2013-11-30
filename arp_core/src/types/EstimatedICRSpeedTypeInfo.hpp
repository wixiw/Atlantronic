#ifndef ESTIMATEDICRSPEEDTYPEINFO_HPP_
#define ESTIMATEDICRSPEEDTYPEINFO_HPP_

#include <rtt/types/StructTypeInfo.hpp>
#include "math/EstimatedICRSpeed.hpp"
#include "math/MathFactory.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

using namespace arp_math;
using namespace RTT;

namespace boost {
namespace serialization {
// The helper function which you write yourself:
template<class Archive>
void serialize( Archive & a, EstimatedICRSpeed& speed, unsigned int) {
        using boost::serialization::make_nvp;
        a & make_nvp("ro", speed.roRef());
        a & make_nvp("phi", speed.phiRef());
        a & make_nvp("delta", speed.deltaRef());
        //TODO à remettre quand un typekit long double sera dispo
        //a & make_nvp("date", speed.dateRef());
}
}
}


//  // Displaying:
//  std::ostream& operator<<(std::ostream& os, const EstimatedICRSpeed& pose)
//  {
//      return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
//  }
//
//  // Reading :
//  std::istream& operator>>(std::istream& is, EstimatedICRSpeed& cd) {
//      char c;
//     return is >> c >> cd;
//  }

EstimatedICRSpeed createEstimatedICRSpeed(double ro, double phi, double delta)
{
    return EstimatedICRSpeed(ICRSpeed(ro,phi,delta));
}

// The RTT helper class which uses the above function behind the scenes:
struct EstimatedICRSpeedTypeInfo: public RTT::types::StructTypeInfo<EstimatedICRSpeed,false>
{
    EstimatedICRSpeedTypeInfo():
        RTT::types::StructTypeInfo<EstimatedICRSpeed,false>("EstimatedICRSpeed")
        {

        }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<EstimatedICRSpeed>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<EstimatedICRSpeed> >( in );
        data->reset();
        data->evaluate();
        EstimatedICRSpeed speed = data->get();
        os << " ro: " << speed.ro() << " m/s +- (cov todo) mm/s ,";
        os << " phi: " << rad2deg(speed.phi()) << " deg +- (cov todo) deg ,";
        os << " delta:" << rad2deg(speed.delta()) << " deg +- (cov todo) deg ,";
        os << " date = " << speed.date() << " sec";
        return os;
    }

    virtual std::istream& read(std::istream& os, base::DataSourceBase::shared_ptr out ) const
    {
        return os;
    }
};

#endif
