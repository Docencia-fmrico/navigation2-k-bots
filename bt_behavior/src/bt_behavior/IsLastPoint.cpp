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

#include "bt_behavior/IsLastPoint.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_behavior
{

IsLastPoint::IsLastPoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
}


BT::NodeStatus
IsLastPoint::tick()
{
  int number_wp;
  int visited_wp;

  config().blackboard->get("number_wp", number_wp);
  config().blackboard->get("visited_wp", visited_wp);

  visited_wp++;

  // Update the number of visited_wp in the blackboard
  config().blackboard->set("visited_wp", visited_wp);

  if (visited_wp == number_wp) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::IsLastPoint>("IsLastPoint");
}
