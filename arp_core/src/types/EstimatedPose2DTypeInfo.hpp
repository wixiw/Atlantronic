#include <rtt/types/StructTypeInfo.hpp>
#include "math/EstimatedPose2D.hpp"
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
      void serialize( Archive & a, EstimatedPose2D& pose, unsigned int) {
         using boost::serialization::make_nvp;
             a & make_nvp("x", pose.xRef());
             a & make_nvp("y", pose.yRef());
             a & make_nvp("theta", pose.hRef());
             //TODO à remettre quand un typekit long double sera dispo
             //a & make_nvp("date", pose.dateRef());
             a & make_nvp("cov", pose.covRef());
      }
    }
  }


//  // Displaying:
//  std::ostream& operator<<(std::ostream& os, const EstimatedPose2D& pose)
//  {
//      return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
//  }
//
//  // Reading :
//  std::istream& operator>>(std::istream& is, EstimatedPose2D& cd) {
//      char c;
//     return is >> c >> cd;
//  }

  EstimatedPose2D createEstimatedPose2D(double x, double y, double angle)
  {
      return MathFactory::createEstimatedPose2D(x,y,angle, 0., Covariance3::Identity());
  }

  // The RTT helper class which uses the above function behind the scenes:
struct EstimatedPose2DTypeInfo: public RTT::types::StructTypeInfo<EstimatedPose2D,false>
{
    EstimatedPose2DTypeInfo():
        RTT::types::StructTypeInfo<EstimatedPose2D,false>("EstimatedPose2D")
    {

    }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<EstimatedPose2D>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<EstimatedPose2D> >( in );
        data->reset();
        data->evaluate();
        EstimatedPose2D pose = data->get();
        os << " x: " << pose.x() << " m +- " << 1.5 * sqrt(pose.cov()(0,0)) * 1000. << " mm ,";
        os << " y: " << pose.y() << " m +- " << 1.5 * sqrt(pose.cov()(1,1)) * 1000. << " mm ,";
        os << " h: " << rad2deg(betweenMinusPiAndPlusPi(pose.angle())) << " deg +- " << 1.5 * rad2deg(sqrt(pose.cov()(2,2))) << " deg ,";
        os << " date = " << pose.date() << " sec";

        return os;
    }

    virtual std::istream& read(std::istream& os, base::DataSourceBase::shared_ptr out ) const
    {
        return os;
    }
};

