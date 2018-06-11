/**
Software License Agreement (BSD)

\file      channel.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
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

#include "roboteq_driver/channel.h"
#include "roboteq_driver/controller.h"

#include "ros/ros.h"
#include "roboteq_msgs/Feedback.h"


namespace roboteq {

Channel::Channel(int channel_num, std::string ns, Controller* controller, int ticks_per_rotation, double gearbox_divider, float max_acceleration, float max_decceleration) :
  channel_num_(channel_num), nh_(ns), controller_(controller), max_rpm_(3000), max_acceleration_(max_acceleration), max_decceleration_(max_decceleration), 
  ticks_per_rotation_(ticks_per_rotation), gearbox_divider_(gearbox_divider)
{
  sub_cmd_ = nh_.subscribe("cmd", 1, &Channel::cmdCallback, this);
  sub_cmd_acc_ = nh_.subscribe("cmd_acc", 1, &Channel::cmdCallbackAcc, this);
  pub_feedback_ = nh_.advertise<roboteq_msgs::Feedback>("feedback", 1);

  // Don't start this timer until we've received the first motion command, otherwise it
  // can interfere with code download on device startup.
  timeout_timer_ = nh_.createTimer(ros::Duration(0.1), &Channel::timeoutCallback, this);
  timeout_timer_.stop();
}

void Channel::cmdCallback(const std_msgs::Float64& command)
{
  // Reset command timeout.
  timeout_timer_.stop();
  timeout_timer_.start();

  // Update mode of motor driver. We send this on each command for redundancy against a
  // lost message, and the MBS script keeps track of changes and updates the control
  // constants accordingly.
  int roboteq_velocity = to_rpm(command.data) / max_rpm_ * 1000.0;

  if(roboteq_velocity > 1000) {
    roboteq_velocity = 1000;
  } else if (roboteq_velocity < -1000) {
    roboteq_velocity = -1000;
  }

  ROS_DEBUG_STREAM("Commanding " << roboteq_velocity << " to motor driver.");
  controller_->command << "G" << channel_num_ << roboteq_velocity << controller_->send;
  controller_->flush();
}

void Channel::cmdCallbackAcc(const roboteq_msgs::SpeedAccelerationCommand& command)
{
  // Reset command timeout.
  timeout_timer_.stop();
  timeout_timer_.start();

  // Update mode of motor driver. We send this on each command for redundancy against a
  // lost message, and the MBS script keeps track of changes and updates the control
  // constants accordingly.

  int roboteq_velocity  = to_rpm(command.cmd) / max_rpm_ * 1000.0;
  int max_acceleration  = command.max_acceleration * max_acceleration_;
  int max_decceleration = command.max_decceleration * max_acceleration_;
  if (max_acceleration <= 0) max_acceleration = max_acceleration_;
  if (max_decceleration <= 0) max_decceleration = max_decceleration_;

  if(roboteq_velocity > 1000) {
    roboteq_velocity = 1000;
  } else if (roboteq_velocity < -1000) {
    roboteq_velocity = -1000;
  }

  ROS_DEBUG_STREAM("Speed: " << roboteq_velocity << " AC: " << max_acceleration << " DC: " << max_decceleration << " to motor driver.");
  controller_->command << "G" << channel_num_ << roboteq_velocity << controller_->send;
  controller_->command << "AC" << channel_num_ << max_acceleration << controller_->send;
  controller_->command << "DC" << channel_num_ << max_decceleration << controller_->send;
  controller_->flush();
}

void Channel::timeoutCallback(const ros::TimerEvent&)
{
  // Sends stop command repeatedly at 10Hz when not being otherwise commanded. Sending
  // repeatedly is a hedge against a lost serial message.
  ROS_DEBUG("Commanding motor to stop due to user command timeout.");
  controller_->command << "G" << channel_num_ << int(0) << controller_->send;
  controller_->flush();
}

void Channel::setMaxRPM(int rpm) {
  if(rpm > 0) max_rpm_ = float(rpm);
}

void Channel::feedbackCallback(std::vector<std::string> fields)
{
  roboteq_msgs::Feedback msg;
  msg.header.stamp = last_feedback_time_ = ros::Time::now();

  // Scale factors as outlined in the relevant portions of the user manual, please
  // see mbs/script.mbs for URL and specific page references.
  try
  {
    msg.cmd_user = from_rpm(boost::lexical_cast<double>(fields[2]));
    msg.cmd_motor = boost::lexical_cast<float>(fields[3]) / 1000.0;
    msg.measured_vel = from_rpm(boost::lexical_cast<double>(fields[4]));
    msg.measured_pos = from_encoder_ticks(boost::lexical_cast<double>(fields[5]));
  }
  catch (std::bad_cast& e)
  {
    ROS_WARN("Failure parsing feedback data. Dropping message.");
    return;
  }
  pub_feedback_.publish(msg);
}

}
