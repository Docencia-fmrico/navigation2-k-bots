// Copyright 2021 Intelligent Robotics Lab
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

#ifndef BT_BEHAVIOR__SEND_WAYPOINT_HPP_
#define BT_BEHAVIOR__SEND_WAYPOINT_HPP_

#include <string>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"


namespace bt_behavior
{

class SendWaypoint : public BT::ActionNodeBase
{
public:
  explicit SendWaypoint(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override;
  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr data_map_);
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr data_map_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> waypoints_;
  int counter;

};

}  // namespace bt_behavior

#endif  // BT_BEHAVIOR__SEND_WAYPOINT_HPP_
