  1#include <functional>
  2#include <memory>
  3#include <thread>
  4
  5#include "action_tutorials_interfaces/action/fibonacci.hpp"
  6#include "rclcpp/rclcpp.hpp"
  7#include "rclcpp_action/rclcpp_action.hpp"
  8#include "rclcpp_components/register_node_macro.hpp"
  9
 10#include "action_tutorials_cpp/visibility_control.h"
 11
 12namespace action_tutorials_cpp
 13{
 14class FibonacciActionServer : public rclcpp::Node
 15{
 16public:
 17  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
 18  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
 19
 20  ACTION_TUTORIALS_CPP_PUBLIC
 21  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
 22  : Node("fibonacci_action_server", options)
 23  {
 24    using namespace std::placeholders;
 25
 26    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
 27      this,
 28      "fibonacci",
 29      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
 30      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
 31      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
 32  }
 33
 34private:
 35  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
 36
 37  rclcpp_action::GoalResponse handle_goal(
 38    const rclcpp_action::GoalUUID & uuid,
 39    std::shared_ptr<const Fibonacci::Goal> goal)
 40  {
 41    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
 42    (void)uuid;
 43    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
 44  }
 45
 46  rclcpp_action::CancelResponse handle_cancel(
 47    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
 48  {
 49    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
 50    (void)goal_handle;
 51    return rclcpp_action::CancelResponse::ACCEPT;
 52  }
 53
 54  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
 55  {
 56    using namespace std::placeholders;
 57    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
 58    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
 59  }
 60
 61  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
 62  {
 63    RCLCPP_INFO(this->get_logger(), "Executing goal");
 64    rclcpp::Rate loop_rate(1);
 65    const auto goal = goal_handle->get_goal();
 66    auto feedback = std::make_shared<Fibonacci::Feedback>();
 67    auto & sequence = feedback->partial_sequence;
 68    sequence.push_back(0);
 69    sequence.push_back(1);
 70    auto result = std::make_shared<Fibonacci::Result>();
 71
 72    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
 73      // Check if there is a cancel request
 74      if (goal_handle->is_canceling()) {
 75        result->sequence = sequence;
 76        goal_handle->canceled(result);
 77        RCLCPP_INFO(this->get_logger(), "Goal canceled");
 78        return;
 79      }
 80      // Update sequence
 81      sequence.push_back(sequence[i] + sequence[i - 1]);
 82      // Publish feedback
 83      goal_handle->publish_feedback(feedback);
 84      RCLCPP_INFO(this->get_logger(), "Publish feedback");
 85
 86      loop_rate.sleep();
 87    }
 88
 89    // Check if goal is done
 90    if (rclcpp::ok()) {
 91      result->sequence = sequence;
 92      goal_handle->succeed(result);
 93      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
 94    }
 95  }
 96};  // class FibonacciActionServer
 97
 98}  // namespace action_tutorials_cpp
 99
100RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)