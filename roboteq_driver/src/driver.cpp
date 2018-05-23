/**
Software License Agreement (BSD)

\file      driver.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
           Mike Irvine <mirvine@clearpathrobotics.com>
           Alexis Paques <alexis.paques@gmail.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.
\copyright Copyright (c) 2015, Unmanned System Ltd., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"

#include "ros/ros.h"

#include <sstream>
#include <string>
#include <iostream>

#include <xmlrpcpp/XmlRpcException.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "roboteq_usb_driver");
  ros::NodeHandle pnh("~"); // Private node handle

  std::string port = "/dev/ttyUSB0";
  int32_t baud = 115200;
  pnh.param<std::string>("port", port, port);
  pnh.param<int32_t>("baud", baud, baud);

  // Interface to motor controller.
  roboteq::Controller controller(port.c_str(), baud);

  // Setup channels.
  if (pnh.hasParam("channels")) {
    XmlRpc::XmlRpcValue channel_namespaces;
    pnh.getParam("channels", channel_namespaces);
    // Config of channels is an array
    ROS_ASSERT(channel_namespaces.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < channel_namespaces.size(); ++i) 
    {
      int encoder_ticks = 24;
      double gearbox_divider = 1;
      std::ostringstream ss;
      // Handle string only = name with 4096 ticks/rotation
      if (channel_namespaces[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
        ss << channel_namespaces[i];
      }

      // Handle dict with optional encoder_ticks
      else if(channel_namespaces[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        try {
          // set encoder ticks with default of 4096 ticks per rotation
          if (channel_namespaces[i].hasMember("encoder_ticks")) {
            if (channel_namespaces[i]["encoder_ticks"].getType() == XmlRpc::XmlRpcValue::TypeInt)
              encoder_ticks = int(channel_namespaces[i]["encoder_ticks"]);
            else if (channel_namespaces[i]["encoder_ticks"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              encoder_ticks = int(channel_namespaces[i]["encoder_ticks"]);
          }
          // set gearbox divider; default is 9
          if (channel_namespaces[i].hasMember("gearbox_divider")) {
            if (channel_namespaces[i]["gearbox_divider"].getType() == XmlRpc::XmlRpcValue::TypeInt)
              gearbox_divider = double(int(channel_namespaces[i]["gearbox_divider"]));
            else if (channel_namespaces[i]["gearbox_divider"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              gearbox_divider = double(channel_namespaces[i]["gearbox_divider"]);
          }

          // set name which has to be a string. Else: MotorID
          if (channel_namespaces[i].hasMember("name")) {
            if (channel_namespaces[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
              ss << channel_namespaces[i]["name"];
            else {
              ss << "motor" <<  (i+1);
              ROS_ERROR_STREAM("Channel array using Dict requires a name for the controller; default to motor" << i+1);
            }
          }
        }
        catch(XmlRpc::XmlRpcException e) {
          ROS_FATAL_STREAM(e.getMessage());
        }
        
      } else {
        ss << "motor" <<  (i+1);
        ROS_ERROR_STREAM("Channel array has to be either a dict or a string. default to motor" << i+1);
      }
      ROS_DEBUG_STREAM("Setting channel " << ss.str() << " with encoders: " << encoder_ticks);
      controller.addChannel(new roboteq::Channel(1 + i, ss.str(), &controller, encoder_ticks, gearbox_divider));
    }
  } else {
    // Default configuration is a single channel in the node's namespace.
    ROS_DEBUG_STREAM("Channels not set: Using default motor1 with 4096 ticks");
    controller.addChannel(new roboteq::Channel(1, "motor1", &controller));
  } 

  ros::AsyncSpinner spinner(1);
  // Attempt to connect and run.
  while (ros::ok()) {
    ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
    controller.connect();
    if (controller.connected()) {
      spinner.start();
      while (controller.connected()) { // Try to reconnect on disconnection
        controller.spinOnce();
      }
      spinner.stop();
    } else {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
      sleep(1);
    }
  }
  spinner.stop();
  ros::waitForShutdown();

  return 0;
}
