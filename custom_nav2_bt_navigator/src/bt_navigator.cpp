// Copyright (c) 2018 Intel Corporation
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

#include "custom_nav2_bt_navigator/bt_navigator.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_bt_navigator/ros_topic_logger.hpp"

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_util::LifecycleNode("bt_navigator", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("bt_xml_filename");

  declare_parameter("plugin_lib_names",
    rclcpp::ParameterValue(std::vector<std::string>({"nav2_behavior_tree_nodes"})));
}

BtNavigator::~BtNavigator()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r", std::string("__node:=") + get_name() + "_client_node",
      "--"});
  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  self_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_, "NavigateToPose");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&BtNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  goal_stat_pub_ = create_publisher<custom_interfaces::msg::GoalFeedback>("topic", 10);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "NavigateToPose", std::bind(&BtNavigator::navigateToPose, this), false);

  // Get the libraries to pull plugins from
  get_parameter("plugin_lib_names", plugin_lib_names_);

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));  // NOLINT
  blackboard_->set<bool>("path_updated", false);  // NOLINT
  blackboard_->set<bool>("initial_pose_received", false);  // NOLINT

  // Get the BT filename to use from the node parameter
  std::string bt_xml_filename;
  get_parameter("bt_xml_filename", bt_xml_filename);

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  xml_string_ = std::string(std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string_.c_str());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // TODO(orduno) Fix the race condition between the worker thread ticking the tree
  //              and the main thread resetting the resources, see #1344

  goal_sub_.reset();
  goal_stat_pub_.reset();
  client_node_.reset();
  self_client_.reset();

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  action_server_.reset();
  plugin_lib_names_.clear();
  xml_string_.clear();
  blackboard_.reset();
  bt_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
BtNavigator::navigateToPose()
{
  initializeGoalPose();

  auto is_canceling = [this]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable. Canceling.");
        return true;
      }

      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server is inactive. Canceling.");
        return true;
      }

      return action_server_->is_cancel_requested();
    };

  // Create the Behavior Tree from the XML input
  BT::Tree tree = bt_->buildTreeFromText(xml_string_, blackboard_);

  RosTopicLogger topic_logger(client_node_, tree);

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested()) {
        RCLCPP_INFO(get_logger(), "Received goal preemption request");
        action_server_->accept_pending_goal();
        initializeGoalPose();
      }
      topic_logger.flush();
    };

  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree, on_loop, is_canceling);

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      action_server_->succeeded_current();
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      action_server_->terminate_current();
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(get_logger(), "Navigation canceled");
      action_server_->terminate_all();
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

void
BtNavigator::initializeGoalPose()
{
  auto goal = action_server_->get_current_goal();

  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Update the goal pose on the blackboard
  blackboard_->set("goal", goal->pose);
}

void
BtNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator