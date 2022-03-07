// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "bt_behavior/Move.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace bt_behavior
{
using namespace std::chrono_literals;
using std::placeholders::_1;

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: bt_behavior::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
    conf)
{
  soundPub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("/commands/sound", 100);
  RCLCPP_WARN(node_->get_logger(), "SOUND---------");
  globalCostmapPub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", 10, std::bind(&Move::CostmapCallback, this, _1));
  RCLCPP_WARN(node_->get_logger(), "MAP-----------");
}

void Move::CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  RCLCPP_WARN(node_->get_logger(), "CALLBACK-----------");
  grid_ = msg;
  /*
  this->resizeMap(grid_->info.width, grid_->info.height, grid_->info.resolution, grid_->info.origin.position.x, grid_->info.origin.position.y);
  */
  std::cout << "mensaje" << grid_->info.width << grid_->info.height << std::endl;
  std::cout << "resolution" << grid_->info.resolution << std::endl;
  std::cout << "origin" << grid_->info.origin.position.x << grid_->info.origin.position.y << std::endl;
  std::cout << "A---------" << std::endl;

  std::cout << "ORIGIN" << grid_->info.origin.position.x  << " | " << grid_->info.origin.position.y << std::endl;
  unsigned int mx = static_cast<unsigned int>((wx_ - this->origin_x_) / grid_->info.resolution);
  unsigned int  my = static_cast<unsigned int>((wy_ - this->origin_y_) / grid_->info.resolution);

  std::cout << "mx = " << mx << " | my = " << my << std::endl;

  unsigned char cost = getCost(6);

  std::cout << "COST = " << cost << std::endl;

}

void
Move::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;

  getInput("goal", goal);
  wx_ = goal.pose.position.x;
  wy_ = goal.pose.position.y;
  RCLCPP_WARN(node_->get_logger(), "Destination [%lf, %lf]\n", goal.pose.position.x, goal.pose.position.y);

  goal_.pose = goal;
}

BT::NodeStatus
Move::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Suceeded");
  kobuki_ros_interfaces::msg::Sound msg;
  msg.value = msg.ON;
  soundPub_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Move::on_aborted(){
  RCLCPP_INFO(node_->get_logger(), "navigation Failled");
  kobuki_ros_interfaces::msg::Sound msg;
  msg.value = msg.ERROR;
  soundPub_->publish(msg);
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bt_behavior::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<bt_behavior::Move>(
    "Move", builder);
}
