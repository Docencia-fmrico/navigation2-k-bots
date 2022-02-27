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

#include "bt_behavior/GetWaypoint.hpp"

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_behavior {

using namespace std::chrono_literals;

GetWaypoint::GetWaypoint(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf)

{
  config().blackboard->get("node", node_);
}

BT::NodeStatus GetWaypoint::tick() {
  int counter_visited; //Counter waypoint visited
  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get("waypoints", waypoints_);
  }
  config().blackboard->get("visited_wp", counter_visited);

  geometry_msgs::msg::PoseStamped waypoint; //Return waypoint
  std::vector<double> coords;
  config().blackboard->get(waypoints_[counter_visited], coords); //Get coords a goal

  waypoint.pose.position.x = coords[0];
  waypoint.pose.position.y = coords[1];

  setOutput("goal",waypoint);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<bt_behavior::GetWaypoint>("GetWaypoint"); }
