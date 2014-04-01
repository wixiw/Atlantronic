#ifndef ARDABSOLUTETIMETYPEINFO_HPP_
#define ARDABSOLUTETIMETYPEINFO_HPP_

#include <rtt/types/StructTypeInfo.hpp>
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>
#include "time/ArdTime.hpp"

using namespace arp_time;
using namespace RTT;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, ArdAbsoluteTime& time, unsigned int) {
         using boost::serialization::make_nvp;
             a & make_nvp("seconds",time);
      }
    }
  }


  // The RTT helper class which uses the above function behind the scenes:
struct ArdAbsoluteTimeTypeInfo: public RTT::types::StructTypeInfo<ArdAbsoluteTime,false>
{
    ArdAbsoluteTimeTypeInfo():
        RTT::types::StructTypeInfo<ArdAbsoluteTime,false>("ArdAbsoluteTime")
    {

    }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<ArdAbsoluteTime>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<ArdAbsoluteTime> >( in );
        data->reset();
        data->evaluate();
        ArdAbsoluteTime time;
        time = data->get();
        return os << ArdAbsoluteTimeToStr(time) << " seconds";
    }
};

#endif
