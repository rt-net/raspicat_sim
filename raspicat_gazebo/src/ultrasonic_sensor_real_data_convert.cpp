// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright 2022 RT Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "raspimouse_msgs/LightSensorValues.h"

namespace ultrasonic_sensor_real_data_convert
{
class usensor_convert final
{
public:
  usensor_convert(ros::NodeHandle& nodeHandle) : pnh_(nodeHandle)
  {
    readParameters();
    initPubSub();
    initTimerCb();
  };

  ~usensor_convert()
  {
    usensorValuePublishTimer_.stop();
  };

private:
  ros::NodeHandle pnh_;
  ros::Publisher usensor_pub_;
  ros::Subscriber usensor_left_front_subscriber_, usensor_left_left_side_subscriber_, usensor_right_front_subscriber_,
      usensor_right_side_subscriber_;
  ros::Timer usensorValuePublishTimer_;
  ros::Duration usensorPublishDuration_;

  sensor_msgs::Range left_front_msg_;
  sensor_msgs::Range left_side_msg_;
  sensor_msgs::Range right_front_msg_;
  sensor_msgs::Range right_side_msg_;

  void readParameters()
  {
    double publishRate;
    pnh_.param("publish_rate", publishRate, 10.0);
    if (publishRate == 0.0)
    {
      usensorPublishDuration_.fromSec(0.0);
      ROS_WARN("The rate for publishing the value of cmd_vel is 0.");
    }
    else
    {
      usensorPublishDuration_.fromSec(1.0 / publishRate);
    }
    ROS_ASSERT(!usensorPublishDuration_.isZero());
  }

  void initPubSub()
  {
    usensor_pub_ = pnh_.advertise<raspimouse_msgs::LightSensorValues>("/lightsensors", 1);

    usensor_left_front_subscriber_ = pnh_.subscribe<sensor_msgs::Range>(
        "/left_front_ultrasonic_sensor", 1, [&](const auto& msg) { left_front_msg_ = *msg; });

    usensor_left_left_side_subscriber_ = pnh_.subscribe<sensor_msgs::Range>(
        "/left_side_ultrasonic_sensor", 1, [&](const auto& msg) { left_side_msg_ = *msg; });

    usensor_right_front_subscriber_ = pnh_.subscribe<sensor_msgs::Range>(
        "/right_front_ultrasonic_sensor", 1, [&](const auto& msg) { right_front_msg_ = *msg; });

    usensor_right_side_subscriber_ = pnh_.subscribe<sensor_msgs::Range>(
        "/right_side_ultrasonic_sensor", 1, [&](const auto& msg) { right_side_msg_ = *msg; });
  }

  void initTimerCb()
  {
    usensorValuePublishTimer_ = pnh_.createTimer(usensorPublishDuration_, &usensor_convert::publishUsensorValue, this);
  }

  void publishUsensorValue(const ros::TimerEvent&)
  {
    raspimouse_msgs::LightSensorValues msg;
    msg.left_forward = limitValue(convetMetetrToMillimeter(left_front_msg_.range));
    msg.left_side = limitValue(convetMetetrToMillimeter(left_side_msg_.range));
    msg.right_forward = limitValue(convetMetetrToMillimeter(right_front_msg_.range));
    msg.right_side = limitValue(convetMetetrToMillimeter(right_side_msg_.range));

    msg.sum_forward = msg.left_forward + msg.right_forward;
    msg.sum_all = msg.sum_forward + msg.left_side + msg.right_side;

    usensor_pub_.publish(msg);
  }

  float limitValue(float range_value)
  {
    if (range_value <= 10)
      range_value = 4095;
    else if (range_value > 10 && range_value <= 19)
      range_value += +20;
    else if (range_value > 20 && range_value <= 29)
      range_value += 10;
    else if (range_value >= 3800)
    {
      range_value = 4095;
    }
    return range_value;
  }

  float convetMetetrToMillimeter(float& range_value)
  {
    return range_value * 1000;
  }
};

}  // namespace ultrasonic_sensor_real_data_convert

int main(int argc, char** argv)
{
  ros::init(argc, argv, "usensor_convert");
  ros::NodeHandle pnh("~");

  ultrasonic_sensor_real_data_convert::usensor_convert usensor_convert(pnh);

  ros::AsyncSpinner spinner(pnh.param("num_callback_threads", 4));
  spinner.start();
  ros::waitForShutdown();
  return 0;
}