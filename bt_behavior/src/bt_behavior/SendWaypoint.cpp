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

#include "bt_behavior/SendWaypoint.hpp"

#include <iostream>
#include <string>

#include "SendWaypoint.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_behavior {

using namespace std::chrono_literals;
using std::placeholders::_1;

SendWaypoint::SendWaypoint(const std::string& xml_tag_name, const std::string& action_name,
                           const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf)

{
  data_map_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&SendWaypoint::MapCallback, node_, _1));

  config().blackboard->get("node", node_);
}

void SendWaypoint::halt() { return; }

BT::NodeStatus SendWaypoint::tick() {
  int counter_visited;  // Counter waypoint visited
  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get("waypoints", waypoints_);
  }
  config().blackboard->get("visited_wp", counter_visited);

  RCLCPP_WARN(node_->get_logger(), "Visitados %d\n", counter_visited);

  geometry_msgs::msg::PoseStamped waypoint;  // Return waypoint
  std::vector<double> coords;
  config().blackboard->get(waypoints_[counter_visited], coords);  // Get coords a goal

  waypoint.pose.position.x = coords[0];
  waypoint.pose.position.y = coords[1];

  setOutput("goal", waypoint);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
// BT_REGISTER_NODES(factory) { factory.registerNodeType<bt_behavior::SendWaypoint>("SendWaypoint");
// }

BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<bt_behavior::SendWaypoint>(name, "get_waypoint", config);
  };

  factory.registerBuilder<bt_behavior::SendWaypoint>("SendWaypoint", builder);
}
