#include <arp_master/DifferentialCommand.h>
#include <arp_master/Obstacle.h>
#include <arp_master/StartColor.h>
#include <arp_master/Start.h>
#include <arp_master/Velocity.h>
#include <arp_master/Odo.h>
#include <arp_master/Pose.h>
#include <arp_master/OrderAction.h>
#include <arp_master/OrderGoal.h>
#include <arp_master/OrderActionGoal.h>
#include <arp_master/OrderResult.h>
#include <arp_master/OrderActionResult.h>
#include <arp_master/OrderFeedback.h>
#include <arp_master/OrderActionFeedback.h>

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;

    /** Declare all factory functions */
            void rtt_ros_addType_DifferentialCommand();
        void rtt_ros_addType_Obstacle();
        void rtt_ros_addType_StartColor();
        void rtt_ros_addType_Start();
        void rtt_ros_addType_Velocity();
        void rtt_ros_addType_Odo();
        void rtt_ros_addType_Pose();
        void rtt_ros_addType_OrderAction();
        void rtt_ros_addType_OrderGoal();
        void rtt_ros_addType_OrderActionGoal();
        void rtt_ros_addType_OrderResult();
        void rtt_ros_addType_OrderActionResult();
        void rtt_ros_addType_OrderFeedback();
        void rtt_ros_addType_OrderActionFeedback();

   
    /**
     * This interface defines the types of the realTime package.
     */
    class ROSarp_masterTypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
          return std::string("ros-")+"arp_master";
      }

      virtual bool loadTypes() {
          // call all factory functions
                  rtt_ros_addType_DifferentialCommand(); // factory function for adding TypeInfo.
        rtt_ros_addType_Obstacle(); // factory function for adding TypeInfo.
        rtt_ros_addType_StartColor(); // factory function for adding TypeInfo.
        rtt_ros_addType_Start(); // factory function for adding TypeInfo.
        rtt_ros_addType_Velocity(); // factory function for adding TypeInfo.
        rtt_ros_addType_Odo(); // factory function for adding TypeInfo.
        rtt_ros_addType_Pose(); // factory function for adding TypeInfo.
        rtt_ros_addType_OrderAction(); // factory function for adding TypeInfo.
        rtt_ros_addType_OrderGoal(); // factory function for adding TypeInfo.
        rtt_ros_addType_OrderActionGoal(); // factory function for adding TypeInfo.
        rtt_ros_addType_OrderResult(); // factory function for adding TypeInfo.
        rtt_ros_addType_OrderActionResult(); // factory function for adding TypeInfo.
        rtt_ros_addType_OrderFeedback(); // factory function for adding TypeInfo.
        rtt_ros_addType_OrderActionFeedback(); // factory function for adding TypeInfo.

          return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSarp_masterTypekitPlugin )

