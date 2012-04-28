#include <rtt/types/StructTypeInfo.hpp>
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>
#include <time.h>

using namespace arp_math;
using namespace RTT;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, timespec& time, unsigned int) {
         using boost::serialization::make_nvp;
             a & make_nvp("seconds",(unsigned int&) time.tv_sec);
             a & make_nvp("nsecs", (unsigned int&) time.tv_nsec);
      }
    }
  }


  // The RTT helper class which uses the above function behind the scenes:
struct TimeSpecTypeInfo: public RTT::types::StructTypeInfo<timespec,false>
{
    TimeSpecTypeInfo():
        RTT::types::StructTypeInfo<timespec,false>("timespec")
    {

    }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<timespec>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<timespec> >( in );
        data->reset();
        data->evaluate();
        timespec time = data->get();
        return os << time.tv_sec << " seconds " << time.tv_nsec << " nano secs";
    }
};

