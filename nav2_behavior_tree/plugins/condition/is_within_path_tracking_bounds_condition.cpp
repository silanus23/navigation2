// Copyright (c) 2025 Berkan Tali
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

#include "nav2_behavior_tree/plugins/condition/is_within_path_tracking_bounds_condition.hpp"

namespace nav2_behavior_tree
{

IsWithinPathTrackingBoundsCondition::IsWithinPathTrackingBoundsCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  last_error_(0.0)
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  tracking_error_sub_ = node_->create_subscription<nav2_msgs::msg::TrackingError>(
    "/tracking_error",
    std::bind(&IsWithinPathTrackingBoundsCondition::trackingErrorCallback, this, std::placeholders::_1),
    rclcpp::SystemDefaultsQoS(),
    callback_group_);

  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");

  RCLCPP_DEBUG(node_->get_logger(), "Initialized IsWithinPathTrackingBoundsCondition BT node");
  RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting for tracking error");
}

void IsWithinPathTrackingBoundsCondition::trackingErrorCallback(
  const nav2_msgs::msg::TrackingError::SharedPtr msg)
{
  last_error_ = msg->tracking_error;
}

void IsWithinPathTrackingBoundsCondition::initialize()
{
  getInput("max_error", max_error);
}

BT::NodeStatus IsWithinPathTrackingBoundsCondition::tick()
{
  callback_group_executor_.spin_all(bt_loop_duration_);

  if (last_error_ <= max_error) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behavior_tree::IsWithinPathTrackingBoundsCondition, BT::ConditionNode)