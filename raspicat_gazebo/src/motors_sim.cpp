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

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace motors_sim {
class motors_sim final {
public:
  motors_sim(ros::NodeHandle &nodeHandle, ros::NodeHandle &private_nodeHandle,
             std::string &initial_motor_power)
      : nh_(nodeHandle), pnh_(private_nodeHandle),
        init_mt_pw_(initial_motor_power) {
    readParameters();
    initServiceServer();
    initPubSub();
    initTimerCb();
    initialMotorPower();
  };

  ~motors_sim() { checkCmdVelTimer_.stop(); };

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher sim_cmd_vel_publisher_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::ServiceServer srv_motor_on_, srv_motor_off_;
  ros::Timer checkCmdVelTimer_;
  ros::Duration checkCmdVelDuration_;
  ros::Time last_cmdvel_;

  geometry_msgs::Twist vel_;
  bool is_on_ = false;
  bool in_cmdvel_ = false;
  std::string init_mt_pw_;

  void readParameters() {
    double publishRate;
    pnh_.param("check_cmdvel_rate", publishRate, 10.0);
    if (publishRate == 0.0) {
      checkCmdVelDuration_.fromSec(0.0);
      ROS_WARN("The rate for checking the value of cmd_vel is 0.");
    } else {
      checkCmdVelDuration_.fromSec(1.0 / publishRate);
    }
    ROS_ASSERT(!checkCmdVelDuration_.isZero());
  }

  void initServiceServer() {
    srv_motor_on_ = nh_.advertiseService<std_srvs::TriggerRequest,
                                         std_srvs::TriggerResponse>(
        "motor_on", [&](auto &req, auto &res) {
          ROS_INFO("Called service motor_on.");
          setPower(true);
          res.message = "Motor On";
          res.success = true;
          return true;
        });

    srv_motor_off_ = nh_.advertiseService<std_srvs::TriggerRequest,
                                          std_srvs::TriggerResponse>(
        "motor_off", [&](auto &req, auto &res) {
          ROS_INFO("Called service motor_off.");
          setPower(false);
          pubZeroSimCmdVel();
          res.message = "Motor Off";
          res.success = true;
          return true;
        });
  }

  void initPubSub() {
    sim_cmd_vel_publisher_ =
        pnh_.advertise<geometry_msgs::Twist>("/sim_cmd_vel", 1);

    cmd_vel_subscriber_ = pnh_.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 1, [&](const auto &msg) {
          vel_ = *msg;
          pubSimCmdVel(*msg);
          last_cmdvel_ = ros::Time::now();
          in_cmdvel_ = true;
        });
  }

  void initTimerCb() {
    checkCmdVelTimer_ = pnh_.createTimer(checkCmdVelDuration_, [&](auto &) {
      if (in_cmdvel_ and
          ros::Time::now().toSec() - last_cmdvel_.toSec() >= 1.0) {
            pubZeroSimCmdVel();
      }
    });
  }

  void initialMotorPower() {
    ROS_INFO("Initial motor_on.");
    setPower(init_mt_pw_ == "on");
  }

  void setPower(bool is_on) { is_on_ = is_on; }

  void pubSimCmdVel(const geometry_msgs::Twist &sim_cmd_vel) {
    geometry_msgs::Twist zero_vel;
    is_on_ ? sim_cmd_vel_publisher_.publish(sim_cmd_vel)
           : sim_cmd_vel_publisher_.publish(zero_vel);
  }

  void pubZeroSimCmdVel() {
    geometry_msgs::Twist zero_vel;
    sim_cmd_vel_publisher_.publish(zero_vel);
  }
};

} // namespace motors_sim

int main(int argc, char **argv) {
  ros::init(argc, argv, "motors_sim");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string onoff;
  if (argc > 1)
    onoff = argv[1];

  motors_sim::motors_sim motors_sim(nh, pnh, onoff);

  ros::AsyncSpinner spinner(pnh.param("num_callback_threads", 4));
  spinner.start();
  ros::waitForShutdown();
  return 0;
}