#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "gtest/gtest.h"
#include "piper_trajectory_bridge/trajectory_bridge.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

using piper_trajectory_bridge::SteadyClock;
using trajectory_msgs::msg::JointTrajectoryPoint;

JointTrajectoryPoint point(
  const std::vector<double> & positions,
  double seconds)
{
  JointTrajectoryPoint message;
  message.positions = positions;
  message.time_from_start =
    piper_trajectory_bridge::duration_message(seconds);
  return message;
}

TEST(TrajectoryBridgeHelpers, InterpolatesFromCurrentStateToFirstPoint)
{
  const std::vector<JointTrajectoryPoint> trajectory{
    point({1.0, -1.0}, 2.0)};

  const auto actual = piper_trajectory_bridge::interpolate_positions(
    trajectory, 1.0, {0.0, 0.0});

  ASSERT_EQ(actual.size(), 2U);
  EXPECT_DOUBLE_EQ(actual[0], 0.5);
  EXPECT_DOUBLE_EQ(actual[1], -0.5);
}

TEST(TrajectoryBridgeHelpers, InterpolatesBetweenPointsAndHoldsFinalPoint)
{
  const std::vector<JointTrajectoryPoint> trajectory{
    point({1.0}, 1.0),
    point({3.0}, 3.0)};

  EXPECT_DOUBLE_EQ(
    piper_trajectory_bridge::interpolate_positions(
      trajectory, 2.0, {0.0})[0],
    2.0);
  EXPECT_DOUBLE_EQ(
    piper_trajectory_bridge::interpolate_positions(
      trajectory, 10.0, {0.0})[0],
    3.0);
}

TEST(TrajectoryBridgeHelpers, RejectsNonIncreasingPointTimes)
{
  const std::vector<JointTrajectoryPoint> trajectory{
    point({1.0}, 1.0),
    point({3.0}, 1.0)};

  EXPECT_THROW(
    piper_trajectory_bridge::interpolate_positions(
      trajectory, 1.0, {0.0}),
    std::invalid_argument);
}

TEST(TrajectoryBridgeHelpers, SkipsExpiredAbsoluteDeadlines)
{
  const auto origin = SteadyClock::time_point{};
  const auto period = std::chrono::milliseconds(100);

  EXPECT_EQ(
    piper_trajectory_bridge::next_command_deadline(
      origin + std::chrono::seconds(10),
      origin + std::chrono::milliseconds(10'350),
      period),
    origin + std::chrono::milliseconds(10'400));
}

TEST(TrajectoryBridgeHelpers, UsesOneHundredEightyHertzByDefault)
{
  EXPECT_DOUBLE_EQ(
    piper_trajectory_bridge::kDefaultCommandRateHz,
    180.0);
  EXPECT_DOUBLE_EQ(
    piper_trajectory_bridge::kJointStatePublishRateHz,
    180.0);
}

TEST(
  TrajectoryBridgeIntegration,
  PreservesActionInterfaceAndOneHundredEightyHertzCadence)
{
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(35);
  rclcpp::init(0, nullptr, init_options);

  const std::string test_namespace =
    "/trajectory_bridge_cpp_test_" +
    std::to_string(SteadyClock::now().time_since_epoch().count());
  rclcpp::NodeOptions bridge_options;
  bridge_options.arguments(
  {
    "--ros-args",
    "-r", "__ns:=" + test_namespace,
  });
  bridge_options.parameter_overrides(
  {
    rclcpp::Parameter("has_gripper", true),
    rclcpp::Parameter("command_topic", "commands"),
    rclcpp::Parameter("state_topic", "raw_states"),
    rclcpp::Parameter("joint_state_topic", "joint_states"),
    rclcpp::Parameter("command_rate", 180.0),
    rclcpp::Parameter("goal_time_tolerance", 0.5),
    rclcpp::Parameter("state_timeout", 1.0),
  });

  auto bridge =
    std::make_shared<piper_trajectory_bridge::PiperTrajectoryBridge>(
    bridge_options);
  auto client_node =
    std::make_shared<rclcpp::Node>("test_client", test_namespace);
  auto raw_state_publisher =
    client_node->create_publisher<sensor_msgs::msg::JointState>(
    "raw_states", 10);

  std::mutex samples_mutex;
  std::vector<SteadyClock::time_point> command_times;
  std::vector<sensor_msgs::msg::JointState> normalized_states;
  std::optional<double> first_command_gripper;
  sensor_msgs::msg::JointState latest_state;
  latest_state.name = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  latest_state.position = std::vector<double>(7, 0.0);
  latest_state.position.back() = 0.02;
  latest_state.velocity = std::vector<double>(7, 0.0);

  auto command_subscription =
    client_node->create_subscription<sensor_msgs::msg::JointState>(
    "commands", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr message) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      command_times.push_back(SteadyClock::now());
      if (!first_command_gripper.has_value()) {
        first_command_gripper = message->position.back();
      }
      latest_state.name = message->name;
      latest_state.position = message->position;
      latest_state.velocity.assign(message->name.size(), 0.0);
    });
  auto normalized_state_subscription =
    client_node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr message) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      normalized_states.push_back(*message);
    });
  auto state_timer = client_node->create_wall_timer(
    std::chrono::microseconds(5'556),
    [&]() {
      sensor_msgs::msg::JointState state;
      {
        std::lock_guard<std::mutex> lock(samples_mutex);
        state = latest_state;
      }
      state.header.stamp = client_node->get_clock()->now();
      raw_state_publisher->publish(state);
    });

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  auto action_client =
    rclcpp_action::create_client<FollowJointTrajectory>(
    client_node,
    "arm_controller/follow_joint_trajectory");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(bridge);
  executor.add_node(client_node);
  std::thread executor_thread([&executor]() {executor.spin();});

  const bool server_ready =
    action_client->wait_for_action_server(std::chrono::seconds(2));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  goal.trajectory.points.push_back(
    point({0.2, -0.1, 0.1, -0.2, 0.1, 0.2}, 1.5));

  std::atomic<std::size_t> feedback_count{0};
  std::atomic<bool> feedback_shape_valid{true};
  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_options;
  send_options.feedback_callback =
    [&](auto, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
      feedback_count.fetch_add(1);
      if (feedback->joint_names.size() != 6 ||
        feedback->desired.positions.size() != 6 ||
        feedback->actual.positions.size() != 6 ||
        feedback->error.positions.size() != 6)
      {
        feedback_shape_valid.store(false);
      }
    };
  auto goal_future = action_client->async_send_goal(goal, send_options);
  const bool goal_response_ready =
    goal_future.wait_for(std::chrono::seconds(2)) ==
    std::future_status::ready;
  auto goal_handle =
    goal_response_ready ? goal_future.get() : nullptr;

  bool result_ready = false;
  rclcpp_action::ResultCode result_code =
    rclcpp_action::ResultCode::UNKNOWN;
  int32_t action_error_code =
    FollowJointTrajectory::Result::INVALID_GOAL;
  if (goal_handle) {
    auto result_future = action_client->async_get_result(goal_handle);
    result_ready =
      result_future.wait_for(std::chrono::seconds(3)) ==
      std::future_status::ready;
    if (result_ready) {
      const auto result = result_future.get();
      result_code = result.code;
      if (result.result) {
        action_error_code = result.result->error_code;
      }
    }
  }

  executor.cancel();
  executor_thread.join();
  executor.remove_node(client_node);
  executor.remove_node(bridge);
  state_timer.reset();
  normalized_state_subscription.reset();
  command_subscription.reset();
  client_node.reset();
  bridge.reset();
  rclcpp::shutdown();

  EXPECT_TRUE(server_ready);
  EXPECT_TRUE(goal_response_ready);
  ASSERT_NE(goal_handle, nullptr);
  EXPECT_TRUE(result_ready);
  EXPECT_EQ(result_code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(
    action_error_code,
    FollowJointTrajectory::Result::SUCCESSFUL);
  EXPECT_GT(feedback_count.load(), 0U);
  EXPECT_TRUE(feedback_shape_valid.load());

  std::vector<double> intervals;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    ASSERT_GE(command_times.size(), 250U);
    ASSERT_TRUE(first_command_gripper.has_value());
    EXPECT_DOUBLE_EQ(*first_command_gripper, 0.02);
    ASSERT_FALSE(normalized_states.empty());
    EXPECT_EQ(normalized_states.back().name.size(), 8U);
    EXPECT_DOUBLE_EQ(normalized_states.back().position[6], 0.02);
    EXPECT_DOUBLE_EQ(normalized_states.back().position[7], -0.02);
    intervals.reserve(command_times.size() - 1);
    for (std::size_t index = 1; index < command_times.size(); ++index) {
      intervals.push_back(
        std::chrono::duration<double>(
          command_times[index] - command_times[index - 1]).count());
    }
  }
  const double mean_interval =
    std::accumulate(intervals.begin(), intervals.end(), 0.0) /
    static_cast<double>(intervals.size());
  const double actual_rate = 1.0 / mean_interval;
  const double maximum_interval =
    *std::max_element(intervals.begin(), intervals.end());

  EXPECT_GE(actual_rate, 171.0);
  EXPECT_LE(actual_rate, 189.0);
  EXPECT_LT(maximum_interval, 0.015);
}

TEST(TrajectoryBridgeIntegration, ExecutesSingleJointGripperTrajectory)
{
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(34);
  rclcpp::init(0, nullptr, init_options);

  const std::string test_namespace =
    "/trajectory_bridge_gripper_test_" +
    std::to_string(SteadyClock::now().time_since_epoch().count());
  rclcpp::NodeOptions bridge_options;
  bridge_options.arguments(
  {
    "--ros-args",
    "-r", "__ns:=" + test_namespace,
  });
  bridge_options.parameter_overrides(
  {
    rclcpp::Parameter("has_gripper", true),
    rclcpp::Parameter("command_topic", "commands"),
    rclcpp::Parameter("state_topic", "raw_states"),
    rclcpp::Parameter("joint_state_topic", "joint_states"),
    rclcpp::Parameter("goal_time_tolerance", 0.5),
    rclcpp::Parameter("state_timeout", 1.0),
  });

  auto bridge =
    std::make_shared<piper_trajectory_bridge::PiperTrajectoryBridge>(
    bridge_options);
  auto client_node =
    std::make_shared<rclcpp::Node>("gripper_test_client", test_namespace);
  auto raw_state_publisher =
    client_node->create_publisher<sensor_msgs::msg::JointState>(
    "raw_states", 10);

  std::mutex state_mutex;
  sensor_msgs::msg::JointState latest_state;
  latest_state.name = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  latest_state.position = std::vector<double>(7, 0.0);
  latest_state.velocity = std::vector<double>(7, 0.0);
  std::optional<double> commanded_gripper;

  auto command_subscription =
    client_node->create_subscription<sensor_msgs::msg::JointState>(
    "commands", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr message) {
      std::lock_guard<std::mutex> lock(state_mutex);
      latest_state.name = message->name;
      latest_state.position = message->position;
      latest_state.velocity.assign(message->name.size(), 0.0);
      commanded_gripper = message->position.back();
    });
  auto state_timer = client_node->create_wall_timer(
    std::chrono::microseconds(5'556),
    [&]() {
      sensor_msgs::msg::JointState state;
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        state = latest_state;
      }
      state.header.stamp = client_node->get_clock()->now();
      raw_state_publisher->publish(state);
    });

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  auto action_client =
    rclcpp_action::create_client<FollowJointTrajectory>(
    client_node,
    "gripper_controller/follow_joint_trajectory");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(bridge);
  executor.add_node(client_node);
  std::thread executor_thread([&executor]() {executor.spin();});

  const bool server_ready =
    action_client->wait_for_action_server(std::chrono::seconds(2));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"joint7"};
  goal.trajectory.points.push_back(point({0.03}, 0.2));
  auto goal_future = action_client->async_send_goal(goal);
  const bool goal_response_ready =
    goal_future.wait_for(std::chrono::seconds(2)) ==
    std::future_status::ready;
  auto goal_handle = goal_response_ready ? goal_future.get() : nullptr;

  bool result_ready = false;
  rclcpp_action::ResultCode result_code =
    rclcpp_action::ResultCode::UNKNOWN;
  if (goal_handle) {
    auto result_future = action_client->async_get_result(goal_handle);
    result_ready =
      result_future.wait_for(std::chrono::seconds(2)) ==
      std::future_status::ready;
    if (result_ready) {
      result_code = result_future.get().code;
    }
  }

  executor.cancel();
  executor_thread.join();
  executor.remove_node(client_node);
  executor.remove_node(bridge);
  state_timer.reset();
  command_subscription.reset();
  client_node.reset();
  bridge.reset();
  rclcpp::shutdown();

  EXPECT_TRUE(server_ready);
  EXPECT_TRUE(goal_response_ready);
  ASSERT_NE(goal_handle, nullptr);
  EXPECT_TRUE(result_ready);
  EXPECT_EQ(result_code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(commanded_gripper.has_value());
  EXPECT_NEAR(*commanded_gripper, 0.03, 1e-6);
}

}  // namespace
