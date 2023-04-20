/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_joint_position_subscriber");

class JointSub : public rclcpp::Node
{
public:
  JointSub(const rclcpp::NodeOptions& options, const std::shared_ptr<rclcpp::Node>& node)
  : Node("moveit2_joint_position_subscriber", options), node_(node)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/desired_joint_states", 10, std::bind(&JointSub::topic_callback, this, std::placeholders::_1));
      
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
      moveit::planning_interface::MoveGroupInterface move_group(node_, "panda_arm");   
      //moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

      //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("panda_arm");
          
      //std::vector<double> joint_group_positions;
      //current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      //joint_group_positions[0] = -1.0;  // radians
      move_group.setJointValueTarget(msg->position);

      move_group.setMaxVelocityScalingFactor(0.05);
      move_group.setMaxAccelerationScalingFactor(0.05);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      move_group.plan(my_plan);
      move_group.execute(my_plan);
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit2_joint_position_subscriber", rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  auto joint_sub = std::make_shared<JointSub>(rclcpp::NodeOptions(), node);
  rclcpp::spin(joint_sub);
  rclcpp::shutdown();
  return 0;
}
