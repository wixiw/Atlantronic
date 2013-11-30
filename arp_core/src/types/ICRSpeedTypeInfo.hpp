#ifndef ICRSPEEDTYPEINFO_HPP_
#define ICRSPEEDTYPEINFO_HPP_

#include <rtt/types/StructTypeInfo.hpp>
#include "math/ICRSpeed.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

using namespace arp_math;
using namespace RTT;

namespace boost
{
namespace serialization
{
// The helper function which you write yourself:
template<class Archive>
void serialize(Archive & a, ICRSpeed& speed, unsigned int)
{
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
//  std::ostream& operator<<(std::ostream& os, const ICRSpeed& pose)
//  {
//      return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
//  }
//
//  // Reading :
//  std::istream& operator>>(std::istream& is, ICRSpeed& cd) {
//      char c;
//     return is >> c >> cd;
//  }

ICRSpeed createICRSpeed(double ro, double phi, double delta)
{
    return ICRSpeed(ICRSpeed(ro, phi, delta));
}

// The RTT helper class which uses the above function behind the scenes:
struct ICRSpeedTypeInfo: public RTT::types::StructTypeInfo<ICRSpeed, false>
{
        ICRSpeedTypeInfo() :
                RTT::types::StructTypeInfo<ICRSpeed, false>("ICRSpeed")
        {

        }

        //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
        //du coup c'est fait a la warrior.
        virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in) const
        {
            internal::DataSource<ICRSpeed>::shared_ptr data =
                    boost::dynamic_pointer_cast<internal::DataSource<ICRSpeed> >(in);
            data->reset();
            data->evaluate();
            ICRSpeed speed = data->get();
            os << " ro: " << speed.ro() << ",";
            os << " phi: " << rad2deg(speed.phi()) << ",";
            os << " delta:" << rad2deg(speed.delta());
            return os;
        }

        virtual std::istream& read(std::istream& os, base::DataSourceBase::shared_ptr out) const
        {
            return os;
        }
};

#endif
