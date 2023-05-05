// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "raspicat_gazebo/raspicat_sim.hpp"

constexpr auto INIT_MOTOR_POWER_PARAM = "initial_motor_power";

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace raspicatsim
{

RaspicatSim::RaspicatSim(const std::string & node_name, bool intra_process_comms)
: motor_power_flag_(false)
  , rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaspicatSim::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring %s node", get_name());

  declare_parameter(INIT_MOTOR_POWER_PARAM, false);

  sim_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("sim_cmd_vel", 10);
  velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&RaspicatSim::velocity_command, this, _1));
  power_service_ = create_service<std_srvs::srv::SetBool>(
    "motor_power", std::bind(&RaspicatSim::handle_motor_power, this, _1, _2, _3));
  watchdog_timer_ = create_wall_timer(60s, std::bind(&RaspicatSim::watchdog, this));

  set_motor_power(false);

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaspicatSim::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Activating %s node", get_name());

  LifecycleNode::on_activate(state);

  motor_power_flag_ = get_parameter(INIT_MOTOR_POWER_PARAM).get_value<bool>();
  set_motor_power(motor_power_flag_);

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaspicatSim::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating %s node", get_name());

  LifecycleNode::on_deactivate(state);

  set_motor_power(false);
  stop_motors();

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaspicatSim::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up %s node", get_name());

  set_motor_power(false);
  stop_motors();
  release_pointers();

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaspicatSim::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down %s node", get_name());

  set_motor_power(false);
  stop_motors();
  release_pointers();

  return CallbackReturn::SUCCESS;
}

void RaspicatSim::velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  watchdog_timer_->reset();
  if (motor_power_flag_) {
    sim_cmd_vel_pub_->publish(*msg);
  } else {
    stop_motors();
  }
}

void RaspicatSim::release_pointers()
{
  sim_cmd_vel_pub_.reset();
  velocity_sub_.reset();
  power_service_.reset();
  watchdog_timer_.reset();
}

void RaspicatSim::handle_motor_power(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  set_motor_power(request->data);
  response->success = true;
  if (request->data) {
    response->message = "Motors are on";
  } else {
    response->message = "Motors are off";
  }
}

void RaspicatSim::watchdog()
{
  RCLCPP_INFO(get_logger(), "Watchdog timeout; stopping motors");
  motor_power_flag_ = false;
  stop_motors();
  watchdog_timer_->cancel();
}

void RaspicatSim::set_motor_power(bool value)
{
  if (value) {
    RCLCPP_INFO(get_logger(), "Turned motors on");
    motor_power_flag_ = true;
    watchdog_timer_->reset();
  } else {
    RCLCPP_INFO(get_logger(), "Turned motors off");
    watchdog_timer_->cancel();
    motor_power_flag_ = false;
    stop_motors();
  }
}

void RaspicatSim::stop_motors()
{
  auto stop_velocity = geometry_msgs::msg::Twist();
  sim_cmd_vel_pub_->publish(stop_velocity);
}
}  // namespace raspicatsim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<raspicatsim::RaspicatSim> rs_node = std::make_shared<raspicatsim::RaspicatSim>(
    "raspicat_sim");

  exe.add_node(rs_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
