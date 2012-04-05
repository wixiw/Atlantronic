#include <rtt/types/StructTypeInfo.hpp>
#include "math/Twist2D.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

using namespace arp_math;
using namespace RTT;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, Twist2D& twist, unsigned int) {
         using boost::serialization::make_nvp;
             a & make_nvp("vx", twist.vxRef());
             a & make_nvp("vy", twist.vyRef());
             a & make_nvp("vtheta", twist.vhRef());
      }
    }
  }


//  // Displaying:
//  std::ostream& operator<<(std::ostream& os, const Twist2D& pose)
//  {
//      return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
//  }
//
//  // Reading :
//  std::istream& operator>>(std::istream& is, Twist2D& cd) {
//      char c;
//     return is >> c >> cd;
//  }

  Twist2D createTwist2D(double vx, double vy, double vangle)
  {
      return Twist2D(vx,vy,vangle);
  }

  // The RTT helper class which uses the above function behind the scenes:
struct Twist2DTypeInfo: public RTT::types::StructTypeInfo<Twist2D,false>
{
    Twist2DTypeInfo():
        RTT::types::StructTypeInfo<Twist2D,false>("Twist2D")
    {

    }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<Twist2D>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<Twist2D> >( in );
        data->reset();
        data->evaluate();
        Twist2D t = data->get();
        return os << "(" << t.vx() << "," << t.vy() << "," << t.vh() << ")";
    }

    virtual std::istream& read(std::istream& os, base::DataSourceBase::shared_ptr out ) const
    {
        return os;
    }
};

