#include <arp_core/Odo.h>
#include <arp_core/Velocity.h>
#include <arp_core/Obstacle.h>
#include <arp_core/Start.h>
#include <arp_core/StartColor.h>
#include <arp_core/Pose.h>
#include <arp_core/DifferentialCommand.h>

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;

    /** Declare all factory functions */
            void rtt_ros_addType_Odo();
        void rtt_ros_addType_Velocity();
        void rtt_ros_addType_Obstacle();
        void rtt_ros_addType_Start();
        void rtt_ros_addType_StartColor();
        void rtt_ros_addType_Pose();
        void rtt_ros_addType_DifferentialCommand();

   
    /**
     * This interface defines the types of the realTime package.
     */
    class ROSarp_coreTypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
          return std::string("ros-")+"arp_core";
      }

      virtual bool loadTypes() {
          // call all factory functions
                  rtt_ros_addType_Odo(); // factory function for adding TypeInfo.
        rtt_ros_addType_Velocity(); // factory function for adding TypeInfo.
        rtt_ros_addType_Obstacle(); // factory function for adding TypeInfo.
        rtt_ros_addType_Start(); // factory function for adding TypeInfo.
        rtt_ros_addType_StartColor(); // factory function for adding TypeInfo.
        rtt_ros_addType_Pose(); // factory function for adding TypeInfo.
        rtt_ros_addType_DifferentialCommand(); // factory function for adding TypeInfo.

          return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSarp_coreTypekitPlugin )

