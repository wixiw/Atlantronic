#include <rtt/types/StructTypeInfo.hpp>
#include "math/Pose2D.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

using namespace arp_math;
using namespace RTT;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, Pose2D& pose, unsigned int) {
         using boost::serialization::make_nvp;
             a & make_nvp("x", pose.xRef());
             a & make_nvp("y", pose.xRef());
             a & make_nvp("theta", pose.hRef());
      }
    }
  }


//  // Displaying:
//  std::ostream& operator<<(std::ostream& os, const Pose2D& pose)
//  {
//      return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
//  }
//
//  // Reading :
//  std::istream& operator>>(std::istream& is, Pose2D& cd) {
//      char c;
//     return is >> c >> cd;
//  }

  Pose2D createPose2D(double x, double y, double angle)
  {
      return Pose2D(x,y,angle);
  }

  // The RTT helper class which uses the above function behind the scenes:
struct Pose2DTypeInfo: public RTT::types::StructTypeInfo<Pose2D,false>
{
    Pose2DTypeInfo():
        RTT::types::StructTypeInfo<Pose2D,false>("Pose2D")
    {

    }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<Pose2D>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<Pose2D> >( in );
        data->reset();
        data->evaluate();
        Pose2D pose = data->get();
        return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
    }

    virtual std::istream& read(std::istream& os, base::DataSourceBase::shared_ptr out ) const
    {
        return os;
    }
};

