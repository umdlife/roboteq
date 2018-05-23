/**
Software License Agreement (BSD)

\file      channel.h
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

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
#ifndef ROBOTEQ_CHANNEL
#define ROBOTEQ_CHANNEL

#include "ros/ros.h"
#include <std_msgs/Float64.h>

namespace roboteq_msgs {
  ROS_DECLARE_MESSAGE(Command);
  ROS_DECLARE_MESSAGE(Feedback);
}

namespace roboteq {

class Controller;

class Channel {
public:
  Channel(int channel_num, std::string ns, Controller* controller, int ticks_per_rotation=24, double gearbox_divider=1);
  void feedbackCallback(std::vector<std::string>);
  void setMaxRPM(int rpm);

protected:
  /**
   * @param x Angular velocity in radians/s.
   * @return Angular velocity in RPM.
   */
  double to_rpm(double x)
  {
    return x * 60 / (2 * M_PI) * gearbox_divider_;
  }

  /**
   * @param x Angular velocity in RPM.
   * @return Angular velocity in rad/s.
   */
  double from_rpm(double x)
  {
    return x * (2 * M_PI) / 60 / gearbox_divider_;
  }

  /**
   * Conversion of radians to hall sensors ticks. Default is assumess 3
   * hall sensors and 8 poles, hence 24.
   *
   * @param x Angular position in radians.
   * @return Angular position in hall sensor ticks.
   */
  double to_encoder_ticks(double x)
  {
    return x * ticks_per_rotation_ / (2 * M_PI) * gearbox_divider_;
  }

  /**
   * Conversion of hall sensors ticks to radians. Default is assumess 3
   * hall sensors and 8 poles, hence 24.
   *
   * @param x Angular position in hall sensor ticks.
   * @return Angular position in radians.
   */
  double from_encoder_ticks(double x)
  {
    return x * (2 * M_PI) / ticks_per_rotation_ / gearbox_divider_;
  }

  void cmdCallback(const std_msgs::Float64&);
  void timeoutCallback(const ros::TimerEvent&);

  ros::NodeHandle nh_;
  boost::shared_ptr<Controller> controller_;
  int channel_num_;
  float max_rpm_;
  int ticks_per_rotation_;
  double gearbox_divider_;

  ros::Subscriber sub_cmd_;
  ros::Publisher pub_feedback_;
  ros::Timer timeout_timer_;

  ros::Time last_feedback_time_;
};

}

#endif
