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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <chrono>

#include "nav2_msgs/msg/tracking_feedback.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_within_path_tracking_bounds_condition.hpp"

class IsWithinPathTrackingBoundsConditionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("test_is_within_path_tracking_bounds");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));

    factory_->registerNodeType<nav2_behavior_tree::IsWithinPathTrackingBoundsCondition>(
      "IsWithinPathTrackingBounds");

    tracking_feedback_pub_ = node_->create_publisher<nav2_msgs::msg::TrackingFeedback>(
      "tracking_feedback",
      rclcpp::SystemDefaultsQoS());
    tracking_feedback_pub_->on_activate();
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    tracking_feedback_pub_.reset();
    node_.reset();
    factory_.reset();
    executor_.reset();
  }

  void publishAndSpin(float position_error, float heading_error)
  {
    nav2_msgs::msg::TrackingFeedback msg;
    msg.position_tracking_error = position_error;
    msg.heading_tracking_error = heading_error;
    tracking_feedback_pub_->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    executor_->spin_some();
  }

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static nav2::Publisher<nav2_msgs::msg::TrackingFeedback>::SharedPtr tracking_feedback_pub_;
};

nav2::LifecycleNode::SharedPtr IsWithinPathTrackingBoundsConditionTestFixture::node_ = nullptr;
rclcpp::executors::SingleThreadedExecutor::SharedPtr
IsWithinPathTrackingBoundsConditionTestFixture::executor_ = nullptr;
BT::NodeConfiguration * IsWithinPathTrackingBoundsConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsWithinPathTrackingBoundsConditionTestFixture::factory_ =
  nullptr;
nav2::Publisher<nav2_msgs::msg::TrackingFeedback>::SharedPtr
IsWithinPathTrackingBoundsConditionTestFixture::tracking_feedback_pub_ = nullptr;

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_both_errors_within_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-0.5, -0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.0, 0.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_position_error_out_of_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(1.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(-1.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_heading_error_out_of_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.5, 0.8);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.5, -0.8);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_both_errors_out_of_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(1.5, 0.8);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(-1.5, -0.8);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_asymmetric_position_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="2.0" max_position_error_right="0.5"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(1.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-0.3, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(2.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(-0.8, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_asymmetric_heading_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="1.0" max_heading_error_left="0.3"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.5, 0.8);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.5, -0.2);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.5, 1.2);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.5, -0.5);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_position_sign_convention)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="2.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.9, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(1.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(-1.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-2.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_heading_sign_convention)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="1.0" max_heading_error_left="0.3"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.5, 0.9);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.5, 1.2);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.5, -0.2);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.5, -0.5);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_negative_parameters)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="-1.0" max_position_error_right="-1.0"
                                       max_heading_error_right="-0.5" max_heading_error_left="-0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-0.5, -0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(1.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.5, 0.8);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_zero_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="0.0" max_position_error_right="0.0"
                                       max_heading_error_right="0.0" max_heading_error_left="0.0"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.0, 0.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.001, 0.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.0, 0.001);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.001, 0.001);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_very_tight_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="0.1" max_position_error_right="0.1"
                                       max_heading_error_right="0.05" max_heading_error_left="0.05"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.05, 0.03);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-0.05, -0.03);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.15, 0.03);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.05, 0.08);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_position_left_only)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="5.0" max_position_error_right="0.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(4.9, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(5.0, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(5.1, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(-0.001, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.0, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_position_right_only)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="0.0" max_position_error_right="5.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(-4.9, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-5.0, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-5.1, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.001, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.0, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_no_feedback_received)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_transition_sequence)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(0.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(1.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.5, 0.3);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.5, 0.8);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  publishAndSpin(0.0, 0.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_boundary_values)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_position_error_left="1.0" max_position_error_right="1.0"
                                       max_heading_error_right="0.5" max_heading_error_left="0.5"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  publishAndSpin(1.0, 0.5);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(-1.0, -0.5);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(1.0, 0.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  publishAndSpin(0.0, 0.5);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
