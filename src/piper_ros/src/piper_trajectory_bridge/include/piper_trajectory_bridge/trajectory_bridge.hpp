#ifndef PIPER_TRAJECTORY_BRIDGE__TRAJECTORY_BRIDGE_HPP_
#define PIPER_TRAJECTORY_BRIDGE__TRAJECTORY_BRIDGE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_tolerance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace piper_trajectory_bridge
{

using SteadyClock = std::chrono::steady_clock;

inline constexpr double kDefaultCommandRateHz = 180.0;
inline constexpr double kJointStatePublishRateHz = 180.0;

double duration_seconds(const builtin_interfaces::msg::Duration & duration);

builtin_interfaces::msg::Duration duration_message(double seconds);

SteadyClock::time_point next_command_deadline(
  SteadyClock::time_point previous_deadline,
  SteadyClock::time_point now,
  SteadyClock::duration period);

std::vector<double> interpolate_positions(
  const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points,
  double elapsed,
  const std::vector<double> & initial_positions);

class PiperTrajectoryBridge : public rclcpp::Node
{
public:
  explicit PiperTrajectoryBridge(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PiperTrajectoryBridge() override;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle =
    rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using ActionServer = rclcpp_action::Server<FollowJointTrajectory>;
  using JointNames = std::vector<std::string>;

  enum class Group
  {
    kArm,
    kGripper,
  };

  struct ActiveTrajectory
  {
    Group group;
    JointNames expected_joints;
    std::shared_ptr<GoalHandle> goal_handle;
    std::shared_ptr<const FollowJointTrajectory::Goal> goal;
    std::vector<double> initial_positions;
    std::unordered_map<std::string, double> path_tolerances;
    std::unordered_map<std::string, double> goal_tolerances;
    SteadyClock::time_point start_time;
    double final_time;
    double convergence_timeout;
    std::optional<SteadyClock::time_point> convergence_start;
  };

  enum class CompletionState
  {
    kSucceeded,
    kAborted,
    kCanceled,
  };

  struct Completion
  {
    std::shared_ptr<GoalHandle> goal_handle;
    CompletionState state;
    FollowJointTrajectory::Result result;
  };

  struct FeedbackDispatch
  {
    std::shared_ptr<GoalHandle> goal_handle;
    FollowJointTrajectory::Feedback feedback;
  };

  struct TickOutput
  {
    bool publish_command{false};
    std::vector<FeedbackDispatch> feedback;
    std::vector<Completion> completions;
  };

  rclcpp_action::GoalResponse handle_goal(
    Group group,
    const JointNames & expected_joints,
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(
    Group group,
    const JointNames & expected_joints,
    const std::shared_ptr<GoalHandle> goal_handle);

  void state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void publish_joint_state();
  sensor_msgs::msg::JointState normalized_joint_state_locked(
    const std::string & frame_id) const;

  void control_loop();
  void process_active_locked(
    std::optional<ActiveTrajectory> & active,
    SteadyClock::time_point now,
    TickOutput & output);
  void complete_active_locked(
    std::optional<ActiveTrajectory> & active,
    CompletionState state,
    int32_t error_code,
    const std::string & error_string,
    TickOutput & output);
  FollowJointTrajectory::Feedback make_feedback_locked(
    const ActiveTrajectory & active,
    const std::vector<double> & desired_positions,
    double elapsed) const;
  sensor_msgs::msg::JointState command_message_locked() const;

  std::optional<std::string> trajectory_validation_error(
    const FollowJointTrajectory::Goal & goal,
    const JointNames & expected_joints) const;
  std::unordered_map<std::string, double> resolve_position_tolerances(
    const std::vector<control_msgs::msg::JointTolerance> & requested,
    const JointNames & joint_names,
    double default_tolerance) const;
  std::optional<std::string> position_tolerance_violation_locked(
    const JointNames & joint_names,
    const std::vector<double> & desired_positions,
    const std::unordered_map<std::string, double> & tolerances,
    SteadyClock::time_point now) const;
  bool joint_is_fresh_locked(
    const std::string & name,
    SteadyClock::time_point now) const;
  bool joint_state_ready_locked(
    const JointNames & joint_names,
    SteadyClock::time_point now) const;
  bool group_reserved_locked(Group group) const;
  void set_group_reserved_locked(Group group, bool value);
  bool joint_is_active_locked(const std::string & name) const;

  static const JointNames & arm_joints();
  static const JointNames & gripper_joints();

  bool has_gripper_;
  bool hold_gripper_command_;
  double command_rate_;
  double gripper_hold_publish_rate_;
  double default_path_tolerance_;
  double default_goal_tolerance_;
  double default_goal_time_tolerance_;
  double state_timeout_;
  JointNames joint_names_;

  std::unordered_map<std::string, double> actual_positions_;
  std::unordered_map<std::string, double> actual_velocities_;
  std::unordered_map<std::string, double> command_positions_;
  std::unordered_map<std::string, double> command_efforts_;
  std::unordered_set<std::string> received_joints_;
  std::unordered_set<std::string> received_velocities_;
  std::unordered_map<std::string, SteadyClock::time_point> last_state_update_;
  std::string latest_state_frame_id_;

  bool arm_reserved_{false};
  bool gripper_reserved_{false};
  bool has_held_gripper_command_{false};
  SteadyClock::time_point last_gripper_hold_publish_time_{};
  std::optional<ActiveTrajectory> active_arm_;
  std::optional<ActiveTrajectory> active_gripper_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    state_subscription_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  ActionServer::SharedPtr arm_action_server_;
  ActionServer::SharedPtr gripper_action_server_;

  mutable std::mutex mutex_;
  std::condition_variable control_wakeup_;
  std::atomic<bool> stop_control_{false};
  std::thread control_thread_;
};

}  // namespace piper_trajectory_bridge

#endif  // PIPER_TRAJECTORY_BRIDGE__TRAJECTORY_BRIDGE_HPP_
