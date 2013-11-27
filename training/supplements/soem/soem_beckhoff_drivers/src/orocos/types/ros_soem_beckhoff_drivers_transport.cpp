
#include <soem_beckhoff_drivers/PSUMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/CommMsg.h>
#include <soem_beckhoff_drivers/CommMsgBig.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <soem_beckhoff_drivers/EncoderOutMsg.h>
#include <soem_beckhoff_drivers/EncoderInMsg.h>

#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSsoem_beckhoff_driversPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
                   if(name == "/soem_beckhoff_drivers/PSUMsg")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::PSUMsg>());
         if(name == "/soem_beckhoff_drivers/DigitalMsg")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::DigitalMsg>());
         if(name == "/soem_beckhoff_drivers/AnalogMsg")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::AnalogMsg>());
         if(name == "/soem_beckhoff_drivers/CommMsg")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::CommMsg>());
         if(name == "/soem_beckhoff_drivers/CommMsgBig")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::CommMsgBig>());
         if(name == "/soem_beckhoff_drivers/EncoderMsg")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::EncoderMsg>());
         if(name == "/soem_beckhoff_drivers/EncoderOutMsg")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::EncoderOutMsg>());
         if(name == "/soem_beckhoff_drivers/EncoderInMsg")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<soem_beckhoff_drivers::EncoderInMsg>());

          return false;
      }
      
      std::string getTransportName() const {
          return "ros";
      }
      
      std::string getTypekitName() const {
          return std::string("ros-")+"soem_beckhoff_drivers";
      }
      std::string getName() const {
          return std::string("rtt-ros-") + "soem_beckhoff_drivers" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSsoem_beckhoff_driversPlugin )
