#include <soem_beckhoff_drivers/boost/DigitalMsg.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSDigitalMsgPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/soem_beckhoff_drivers/DigitalMsg")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::DigitalMsg>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"DigitalMsg";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "DigitalMsg" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSDigitalMsgPlugin )
