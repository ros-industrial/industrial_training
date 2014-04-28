#include <soem_beckhoff_drivers/boost/EncoderMsg.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSEncoderMsgPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/soem_beckhoff_drivers/EncoderMsg")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::EncoderMsg>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"EncoderMsg";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "EncoderMsg" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSEncoderMsgPlugin )
