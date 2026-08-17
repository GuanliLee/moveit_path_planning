#include "piper_trajectory_bridge/trajectory_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"

namespace piper_trajectory_bridge
{
namespace
{

double time_seconds(const builtin_interfaces::msg::Time & time)
{
  return static_cast<double>(time.sec) +
         static_cast<double>(time.nanosec) / 1'000'000'000.0;
}

bool same_joint_set(
  const std::vector<std::string> & actual,
  const std::vector<std::string> & expected)
{
  if (actual.size() != expected.size()) {
    return false;
  }
  auto sorted_actual = actual;
  auto sorted_expected = expected;
  std::sort(sorted_actual.begin(), sorted_actual.end());
  std::sort(sorted_expected.begin(), sorted_expected.end());
  return sorted_actual == sorted_expected;
}

std::string joint_list(const std::vector<std::string> & names)
{
  std::ostringstream stream;
  stream << '[';
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << names[index];
  }
  stream << ']';
  return stream.str();
}

}  // namespace

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) / 1'000'000'000.0;
}

builtin_interfaces::msg::Duration duration_message(double seconds)
{
  seconds = std::max(0.0, seconds);
  const auto whole_seconds = static_cast<int32_t>(std::floor(seconds));
  auto nanoseconds = static_cast<uint32_t>(
    std::llround((seconds - static_cast<double>(whole_seconds)) *
    1'000'000'000.0));

  builtin_interfaces::msg::Duration duration;
  duration.sec = whole_seconds;
  if (nanoseconds == 1'000'000'000U) {
    duration.sec += 1;
    nanoseconds = 0;
  }
  duration.nanosec = nanoseconds;
  return duration;
}

SteadyClock::time_point next_command_deadline(
  SteadyClock::time_point previous_deadline,
  SteadyClock::time_point now,
  SteadyClock::duration period)
{
  auto deadline = previous_deadline + period;
  if (deadline <= now) {
    const auto overdue = now - deadline;
    const auto skipped_periods = overdue / period + 1;
    deadline += period * skipped_periods;
  }
  return deadline;
}

std::vector<double> interpolate_positions(
  const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points,
  double elapsed,
  const std::vector<double> & initial_positions)
{
  if (points.empty()) {
    throw std::invalid_argument("trajectory must contain at least one point");
  }
  if (initial_positions.size() != points.front().positions.size()) {
    throw std::invalid_argument(
            "initial positions must match trajectory positions");
  }

  std::vector<double> point_times;
  point_times.reserve(points.size());
  for (const auto & point : points) {
    point_times.push_back(duration_seconds(point.time_from_start));
  }
  if (point_times.front() < 0.0) {
    throw std::invalid_argument(
            "trajectory point times must be non-negative and increasing");
  }
  for (std::size_t index = 1; index < point_times.size(); ++index) {
    if (point_times[index] <= point_times[index - 1]) {
      throw std::invalid_argument(
              "trajectory point times must be non-negative and increasing");
    }
  }

  const double first_time = point_times.front();
  if (elapsed <= first_time) {
    if (first_time == 0.0) {
      return points.front().positions;
    }
    const double ratio = std::max(0.0, elapsed) / first_time;
    std::vector<double> interpolated(initial_positions.size());
    for (std::size_t index = 0; index < interpolated.size(); ++index) {
      interpolated[index] =
        initial_positions[index] +
        ratio * (points.front().positions[index] - initial_positions[index]);
    }
    return interpolated;
  }

  for (std::size_t index = 1; index < points.size(); ++index) {
    if (elapsed <= point_times[index]) {
      const double ratio =
        (elapsed - point_times[index - 1]) /
        (point_times[index] - point_times[index - 1]);
      std::vector<double> interpolated(points[index].positions.size());
      for (std::size_t joint = 0; joint < interpolated.size(); ++joint) {
        interpolated[joint] =
          points[index - 1].positions[joint] +
          ratio *
          (points[index].positions[joint] -
          points[index - 1].positions[joint]);
      }
      return interpolated;
    }
  }

  return points.back().positions;
}

const PiperTrajectoryBridge::JointNames &
PiperTrajectoryBridge::arm_joints()
{
  static const JointNames joints{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  return joints;
}

const PiperTrajectoryBridge::JointNames &
PiperTrajectoryBridge::gripper_joints()
{
  static const JointNames joints{"joint7"};
  return joints;
}

PiperTrajectoryBridge::PiperTrajectoryBridge(
  const rclcpp::NodeOptions & options)
: Node("piper_trajectory_bridge", options)
{
  has_gripper_ = declare_parameter<bool>("has_gripper", true);
  hold_gripper_command_ =
    declare_parameter<bool>("hold_gripper_command", false);
  const std::string command_topic =
    declare_parameter<std::string>("command_topic", "joint_ctrl_single");
  const std::string state_topic =
    declare_parameter<std::string>("state_topic", "joint_states_raw");
  const std::string joint_state_topic =
    declare_parameter<std::string>("joint_state_topic", "joint_states");
  command_rate_ =
    declare_parameter<double>("command_rate", kDefaultCommandRateHz);
  gripper_hold_publish_rate_ =
    declare_parameter<double>("gripper_hold_publish_rate", 5.0);
  default_path_tolerance_ =
    declare_parameter<double>("path_tolerance", 0.0);
  default_goal_tolerance_ =
    declare_parameter<double>("goal_tolerance", 0.02);
  default_goal_time_tolerance_ =
    declare_parameter<double>("goal_time_tolerance", 5.0);
  state_timeout_ = declare_parameter<double>("state_timeout", 1.0);

  const auto positive_finite = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
  const auto nonnegative_finite = [](double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (!positive_finite(command_rate_)) {
    throw std::invalid_argument(
            "command_rate must be finite and greater than zero");
  }
  if (!nonnegative_finite(gripper_hold_publish_rate_)) {
    throw std::invalid_argument(
            "gripper_hold_publish_rate must be finite and non-negative");
  }
  if (!nonnegative_finite(default_path_tolerance_)) {
    throw std::invalid_argument(
            "path_tolerance must be finite and non-negative");
  }
  if (!nonnegative_finite(default_goal_tolerance_)) {
    throw std::invalid_argument(
            "goal_tolerance must be finite and non-negative");
  }
  if (!nonnegative_finite(default_goal_time_tolerance_)) {
    throw std::invalid_argument(
            "goal_time_tolerance must be finite and non-negative");
  }
  if (!nonnegative_finite(state_timeout_)) {
    throw std::invalid_argument(
            "state_timeout must be finite and non-negative");
  }
  if (state_topic == joint_state_topic) {
    throw std::invalid_argument(
            "state_topic and joint_state_topic must differ to avoid a feedback loop");
  }

  joint_names_ = arm_joints();
  if (has_gripper_) {
    joint_names_.push_back("joint7");
  }
  for (const auto & name : joint_names_) {
    actual_positions_[name] = 0.0;
    actual_velocities_[name] = 0.0;
    command_positions_[name] = 0.0;
    command_efforts_[name] = 0.0;
  }

  command_publisher_ =
    create_publisher<sensor_msgs::msg::JointState>(command_topic, 10);
  joint_state_publisher_ =
    create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
  state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    state_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(
      &PiperTrajectoryBridge::state_callback, this,
      std::placeholders::_1));

  const auto state_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / kJointStatePublishRateHz));
  joint_state_timer_ = create_wall_timer(
    state_period,
    std::bind(&PiperTrajectoryBridge::publish_joint_state, this));

  arm_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this,
    "arm_controller/follow_joint_trajectory",
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
      return handle_goal(Group::kArm, arm_joints(), uuid, std::move(goal));
    },
    std::bind(
      &PiperTrajectoryBridge::handle_cancel, this,
      std::placeholders::_1),
    [this](const std::shared_ptr<GoalHandle> goal_handle) {
      handle_accepted(Group::kArm, arm_joints(), goal_handle);
    });

  if (has_gripper_) {
    gripper_action_server_ =
      rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "gripper_controller/follow_joint_trajectory",
      [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
      {
        return handle_goal(
          Group::kGripper, gripper_joints(), uuid, std::move(goal));
      },
      std::bind(
        &PiperTrajectoryBridge::handle_cancel, this,
        std::placeholders::_1),
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        handle_accepted(Group::kGripper, gripper_joints(), goal_handle);
      });
  }

  control_thread_ =
    std::thread(&PiperTrajectoryBridge::control_loop, this);

  RCLCPP_INFO(
    get_logger(),
    "Trajectory bridge ready: %s -> %s, commands -> %s",
    state_topic.c_str(), joint_state_topic.c_str(), command_topic.c_str());
}

PiperTrajectoryBridge::~PiperTrajectoryBridge()
{
  stop_control_.store(true);
  control_wakeup_.notify_all();
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

rclcpp_action::GoalResponse PiperTrajectoryBridge::handle_goal(
  Group group,
  const JointNames & expected_joints,
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  const auto validation_error =
    trajectory_validation_error(*goal, expected_joints);
  if (validation_error.has_value()) {
    RCLCPP_ERROR(
      get_logger(), "Rejected trajectory: %s",
      validation_error->c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto now = SteadyClock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> missing_joints;
  for (const auto & name : joint_names_) {
    if (received_joints_.count(name) == 0 ||
      !joint_is_fresh_locked(name, now))
    {
      missing_joints.push_back(name);
    }
  }
  if (!missing_joints.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Rejected trajectory: no fresh hardware state for %s",
      joint_list(missing_joints).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (group_reserved_locked(group)) {
    RCLCPP_WARN(
      get_logger(),
      "Rejected trajectory: this controller already has an active goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  set_group_reserved_locked(group, true);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PiperTrajectoryBridge::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PiperTrajectoryBridge::handle_accepted(
  Group group,
  const JointNames & expected_joints,
  const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const double final_time =
    duration_seconds(goal->trajectory.points.back().time_from_start);
  auto start_time = SteadyClock::now();

  const double requested_start = time_seconds(goal->trajectory.header.stamp);
  if (requested_start > 0.0) {
    const double ros_now = get_clock()->now().seconds();
    const double start_delay = requested_start - ros_now;
    if (-start_delay >= final_time) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        set_group_reserved_locked(group, false);
      }
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code =
        FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP;
      result->error_string = "trajectory end time is in the past";
      goal_handle->abort(result);
      return;
    }
    start_time += std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(start_delay));
  }

  const double requested_goal_time =
    duration_seconds(goal->goal_time_tolerance);
  const double convergence_timeout =
    requested_goal_time == 0.0 ? default_goal_time_tolerance_ :
    requested_goal_time == -1.0 ? 0.0 : requested_goal_time;

  ActiveTrajectory active{
    group,
    expected_joints,
    goal_handle,
    goal,
    {},
    resolve_position_tolerances(
      goal->path_tolerance,
      goal->trajectory.joint_names,
      default_path_tolerance_),
    resolve_position_tolerances(
      goal->goal_tolerance,
      goal->trajectory.joint_names,
      default_goal_tolerance_),
    start_time,
    final_time,
    convergence_timeout,
    std::nullopt};

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active.initial_positions.reserve(goal->trajectory.joint_names.size());
    for (const auto & name : goal->trajectory.joint_names) {
      active.initial_positions.push_back(actual_positions_.at(name));
    }
    for (const auto & name : expected_joints) {
      command_positions_[name] = actual_positions_.at(name);
    }
    if (group == Group::kArm) {
      active_arm_ = std::move(active);
    } else {
      active_gripper_ = std::move(active);
    }
  }
}

void PiperTrajectoryBridge::state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  const auto update_time = SteadyClock::now();
  const bool has_velocity =
    message->velocity.size() == message->name.size();

  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    if (index >= message->position.size()) {
      continue;
    }

    std::string name = message->name[index];
    if (name == "gripper") {
      name = "joint7";
    }
    double position = message->position[index];
    double velocity_sign = 1.0;
    if (name == "joint8") {
      name = "joint7";
      position = -position;
      velocity_sign = -1.0;
    }
    if (actual_positions_.count(name) == 0 || !std::isfinite(position)) {
      continue;
    }

    actual_positions_[name] = position;
    received_joints_.insert(name);
    last_state_update_[name] = update_time;
    if (!joint_is_active_locked(name) &&
      !(hold_gripper_command_ && has_held_gripper_command_ && name == "joint7"))
    {
      command_positions_[name] = position;
    }

    if (has_velocity) {
      const double velocity = velocity_sign * message->velocity[index];
      if (std::isfinite(velocity)) {
        actual_velocities_[name] = velocity;
        received_velocities_.insert(name);
      }
    }
  }
  latest_state_frame_id_ = message->header.frame_id;
}

void PiperTrajectoryBridge::publish_joint_state()
{
  sensor_msgs::msg::JointState normalized_state;
  {
    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!joint_state_ready_locked(joint_names_, now)) {
      return;
    }
    normalized_state =
      normalized_joint_state_locked(latest_state_frame_id_);
  }
  joint_state_publisher_->publish(normalized_state);
}

sensor_msgs::msg::JointState
PiperTrajectoryBridge::normalized_joint_state_locked(
  const std::string & frame_id) const
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = get_clock()->now();
  message.header.frame_id = frame_id;
  message.name = arm_joints();
  message.position.reserve(has_gripper_ ? 8 : 6);
  for (const auto & name : arm_joints()) {
    message.position.push_back(actual_positions_.at(name));
  }
  if (has_gripper_) {
    const double gripper_position = actual_positions_.at("joint7");
    message.name.push_back("joint7");
    message.name.push_back("joint8");
    message.position.push_back(gripper_position);
    message.position.push_back(-gripper_position);
  }

  const bool has_all_velocities = std::all_of(
    joint_names_.begin(), joint_names_.end(),
    [this](const std::string & name) {
      return received_velocities_.count(name) != 0;
    });
  if (has_all_velocities) {
    message.velocity.reserve(has_gripper_ ? 8 : 6);
    for (const auto & name : arm_joints()) {
      message.velocity.push_back(actual_velocities_.at(name));
    }
    if (has_gripper_) {
      const double gripper_velocity = actual_velocities_.at("joint7");
      message.velocity.push_back(gripper_velocity);
      message.velocity.push_back(-gripper_velocity);
    }
  }
  return message;
}

void PiperTrajectoryBridge::control_loop()
{
  const auto period = std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(1.0 / command_rate_));
  const auto gripper_hold_period =
    gripper_hold_publish_rate_ > 0.0 ?
    std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(1.0 / gripper_hold_publish_rate_)) :
    SteadyClock::duration::max();
  auto next_deadline = SteadyClock::now();

  while (rclcpp::ok() && !stop_control_.load()) {
    TickOutput output;
    std::optional<sensor_msgs::msg::JointState> command;
    {
      const auto now = SteadyClock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      process_active_locked(active_arm_, now, output);
      process_active_locked(active_gripper_, now, output);
      if (hold_gripper_command_ && has_gripper_ && has_held_gripper_command_ &&
        !active_arm_.has_value() && !active_gripper_.has_value() &&
        gripper_hold_publish_rate_ > 0.0 &&
        (last_gripper_hold_publish_time_ == SteadyClock::time_point{} ||
        now - last_gripper_hold_publish_time_ >= gripper_hold_period))
      {
        output.publish_command = true;
        last_gripper_hold_publish_time_ = now;
      }
      if (output.publish_command) {
        command = command_message_locked();
      }
    }

    if (command.has_value()) {
      command_publisher_->publish(*command);
    }
    for (auto & dispatch : output.feedback) {
      dispatch.goal_handle->publish_feedback(
        std::make_shared<FollowJointTrajectory::Feedback>(
          std::move(dispatch.feedback)));
    }
    for (auto & completion : output.completions) {
      auto result = std::make_shared<FollowJointTrajectory::Result>(
        std::move(completion.result));
      if (completion.state == CompletionState::kSucceeded) {
        completion.goal_handle->succeed(result);
      } else if (completion.state == CompletionState::kCanceled) {
        completion.goal_handle->canceled(result);
      } else {
        completion.goal_handle->abort(result);
      }
    }

    const auto now_after_publish = SteadyClock::now();
    next_deadline =
      next_command_deadline(next_deadline, now_after_publish, period);
    std::unique_lock<std::mutex> lock(mutex_);
    control_wakeup_.wait_until(
      lock, next_deadline,
      [this]() {return stop_control_.load();});
  }
}

void PiperTrajectoryBridge::process_active_locked(
  std::optional<ActiveTrajectory> & active,
  SteadyClock::time_point now,
  TickOutput & output)
{
  if (!active.has_value()) {
    return;
  }
  if (active->goal_handle->is_canceling()) {
    for (const auto & name : active->expected_joints) {
      command_positions_[name] = actual_positions_.at(name);
    }
    if (active->group == Group::kGripper) {
      has_held_gripper_command_ = false;
    }
    output.publish_command = true;
    complete_active_locked(
      active,
      CompletionState::kCanceled,
      FollowJointTrajectory::Result::SUCCESSFUL,
      "trajectory canceled; holding measured position",
      output);
    return;
  }
  if (now < active->start_time) {
    return;
  }

  const double elapsed =
    std::chrono::duration<double>(now - active->start_time).count();
  const bool reached_end = elapsed >= active->final_time;
  const auto desired = interpolate_positions(
    active->goal->trajectory.points,
    elapsed,
    active->initial_positions);

  if (!active->convergence_start.has_value()) {
    const auto violation = position_tolerance_violation_locked(
      active->goal->trajectory.joint_names,
      desired,
      active->path_tolerances,
      now);
    if (violation.has_value()) {
      complete_active_locked(
        active,
        CompletionState::kAborted,
        FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
        *violation,
        output);
      return;
    }
  }

  for (std::size_t index = 0;
    index < active->goal->trajectory.joint_names.size(); ++index)
  {
    const auto & name = active->goal->trajectory.joint_names[index];
    command_positions_[name] = desired[index];
    const auto & final_effort =
      active->goal->trajectory.points.back().effort;
    if (final_effort.size() == active->goal->trajectory.joint_names.size()) {
      command_efforts_[name] = final_effort[index];
    }
  }
  if (active->group == Group::kGripper) {
    has_held_gripper_command_ = true;
  }
  output.publish_command = true;
  output.feedback.push_back(
    FeedbackDispatch{
      active->goal_handle,
      make_feedback_locked(*active, desired, elapsed)});

  if (!reached_end) {
    return;
  }
  if (!active->convergence_start.has_value()) {
    active->convergence_start = now;
  }

  const auto goal_violation = position_tolerance_violation_locked(
    active->goal->trajectory.joint_names,
    active->goal->trajectory.points.back().positions,
    active->goal_tolerances,
    now);
  if (!goal_violation.has_value()) {
    complete_active_locked(
      active,
      CompletionState::kSucceeded,
      FollowJointTrajectory::Result::SUCCESSFUL,
      "trajectory completed",
      output);
    return;
  }

  const double convergence_elapsed =
    std::chrono::duration<double>(
    now - *active->convergence_start).count();
  if (convergence_elapsed >= active->convergence_timeout) {
    complete_active_locked(
      active,
      CompletionState::kAborted,
      FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
      "hardware feedback did not reach the requested goal before timeout",
      output);
  }
}

void PiperTrajectoryBridge::complete_active_locked(
  std::optional<ActiveTrajectory> & active,
  CompletionState state,
  int32_t error_code,
  const std::string & error_string,
  TickOutput & output)
{
  FollowJointTrajectory::Result result;
  result.error_code = error_code;
  result.error_string = error_string;
  output.completions.push_back(
    Completion{active->goal_handle, state, std::move(result)});
  set_group_reserved_locked(active->group, false);
  active.reset();
}

PiperTrajectoryBridge::FollowJointTrajectory::Feedback
PiperTrajectoryBridge::make_feedback_locked(
  const ActiveTrajectory & active,
  const std::vector<double> & desired_positions,
  double elapsed) const
{
  FollowJointTrajectory::Feedback feedback;
  feedback.header.stamp = get_clock()->now();
  feedback.joint_names = active.goal->trajectory.joint_names;
  feedback.desired.positions = desired_positions;
  feedback.desired.time_from_start = duration_message(elapsed);
  feedback.actual.positions.reserve(feedback.joint_names.size());
  for (const auto & name : feedback.joint_names) {
    feedback.actual.positions.push_back(actual_positions_.at(name));
  }

  const bool has_velocity = std::all_of(
    feedback.joint_names.begin(), feedback.joint_names.end(),
    [this](const std::string & name) {
      return received_velocities_.count(name) != 0;
    });
  if (has_velocity) {
    feedback.actual.velocities.reserve(feedback.joint_names.size());
    for (const auto & name : feedback.joint_names) {
      feedback.actual.velocities.push_back(actual_velocities_.at(name));
    }
  }
  feedback.actual.time_from_start = duration_message(elapsed);
  feedback.error.positions.reserve(feedback.joint_names.size());
  for (std::size_t index = 0; index < desired_positions.size(); ++index) {
    feedback.error.positions.push_back(
      desired_positions[index] - feedback.actual.positions[index]);
  }
  feedback.error.time_from_start = duration_message(elapsed);
  return feedback;
}

sensor_msgs::msg::JointState
PiperTrajectoryBridge::command_message_locked() const
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = get_clock()->now();
  message.name = joint_names_;
  message.position.reserve(joint_names_.size());
  for (const auto & name : joint_names_) {
    message.position.push_back(command_positions_.at(name));
  }
  message.effort.reserve(joint_names_.size());
  for (const auto & name : joint_names_) {
    message.effort.push_back(command_efforts_.at(name));
  }
  return message;
}

std::optional<std::string>
PiperTrajectoryBridge::trajectory_validation_error(
  const FollowJointTrajectory::Goal & goal,
  const JointNames & expected_joints) const
{
  const auto & trajectory = goal.trajectory;
  if (!same_joint_set(trajectory.joint_names, expected_joints)) {
    return "expected joints " + joint_list(expected_joints) +
           ", got " + joint_list(trajectory.joint_names);
  }
  if (trajectory.points.empty()) {
    return "trajectory must contain at least one point";
  }
  if (!goal.multi_dof_trajectory.joint_names.empty() ||
    !goal.multi_dof_trajectory.points.empty())
  {
    return "multi-DOF trajectories are not supported";
  }
  if (!goal.component_path_tolerance.empty() ||
    !goal.component_goal_tolerance.empty())
  {
    return "component tolerances are not supported";
  }

  const std::size_t joint_count = trajectory.joint_names.size();
  double previous_time = -1.0;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points[index];
    const double point_time = duration_seconds(point.time_from_start);
    if (point_time < 0.0 || (index > 0 && point_time <= previous_time)) {
      return "trajectory point times must be non-negative and increasing";
    }
    previous_time = point_time;

    if (point.positions.size() != joint_count) {
      return "every point must define all joint positions";
    }
    const std::array<std::pair<const char *, const std::vector<double> *>, 4>
    fields{{
      {"positions", &point.positions},
      {"velocities", &point.velocities},
      {"accelerations", &point.accelerations},
      {"effort", &point.effort},
    }};
    for (const auto & [field_name, values] : fields) {
      if (std::string(field_name) != "positions" &&
        !values->empty() && values->size() != joint_count)
      {
        return "point " + std::string(field_name) +
               " must be empty or match joint_names";
      }
      if (std::any_of(
          values->begin(), values->end(),
          [](double value) {return !std::isfinite(value);}))
      {
        return "point " + std::string(field_name) +
               " must contain only finite values";
      }
    }
  }

  try {
    resolve_position_tolerances(
      goal.path_tolerance,
      trajectory.joint_names,
      default_path_tolerance_);
    resolve_position_tolerances(
      goal.goal_tolerance,
      trajectory.joint_names,
      default_goal_tolerance_);
  } catch (const std::invalid_argument & error) {
    return error.what();
  }

  const double goal_time_tolerance =
    duration_seconds(goal.goal_time_tolerance);
  if (goal_time_tolerance < 0.0 && goal_time_tolerance != -1.0) {
    return "goal_time_tolerance must be non-negative, zero, or -1";
  }
  return std::nullopt;
}

std::unordered_map<std::string, double>
PiperTrajectoryBridge::resolve_position_tolerances(
  const std::vector<control_msgs::msg::JointTolerance> & requested,
  const JointNames & joint_names,
  double default_tolerance) const
{
  std::unordered_map<std::string, double> tolerances;
  for (const auto & name : joint_names) {
    tolerances[name] = default_tolerance;
  }
  for (const auto & item : requested) {
    if (tolerances.count(item.name) == 0) {
      throw std::invalid_argument(
              "unknown tolerance joint '" + item.name + "'");
    }
    if (!std::isfinite(item.position) ||
      !std::isfinite(item.velocity) ||
      !std::isfinite(item.acceleration))
    {
      throw std::invalid_argument(
              item.name + " has a non-finite tolerance");
    }
    if ((item.velocity != 0.0 && item.velocity != -1.0) ||
      (item.acceleration != 0.0 && item.acceleration != -1.0))
    {
      throw std::invalid_argument(
              "only position tolerances are supported");
    }

    if (item.position > 0.0) {
      tolerances[item.name] = item.position;
    } else if (item.position == -1.0) {
      tolerances[item.name] = std::numeric_limits<double>::infinity();
    } else if (item.position < 0.0) {
      throw std::invalid_argument(
              item.name + " has an invalid position tolerance");
    }
  }
  return tolerances;
}

std::optional<std::string>
PiperTrajectoryBridge::position_tolerance_violation_locked(
  const JointNames & joint_names,
  const std::vector<double> & desired_positions,
  const std::unordered_map<std::string, double> & tolerances,
  SteadyClock::time_point now) const
{
  if (!joint_state_ready_locked(joint_names, now)) {
    return "hardware joint state is missing or stale";
  }
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const auto & name = joint_names[index];
    const double error =
      std::abs(actual_positions_.at(name) - desired_positions[index]);
    const double tolerance = tolerances.at(name);
    if (tolerance > 0.0 && error > tolerance) {
      std::ostringstream stream;
      stream << name << " position error " << error
             << " exceeds tolerance " << tolerance;
      return stream.str();
    }
  }
  return std::nullopt;
}

bool PiperTrajectoryBridge::joint_is_fresh_locked(
  const std::string & name,
  SteadyClock::time_point now) const
{
  const auto found = last_state_update_.find(name);
  if (found == last_state_update_.end()) {
    return false;
  }
  return state_timeout_ == 0.0 ||
         std::chrono::duration<double>(now - found->second).count() <=
         state_timeout_;
}

bool PiperTrajectoryBridge::joint_state_ready_locked(
  const JointNames & joint_names,
  SteadyClock::time_point now) const
{
  return std::all_of(
    joint_names.begin(), joint_names.end(),
    [this, now](const std::string & name) {
      return received_joints_.count(name) != 0 &&
             joint_is_fresh_locked(name, now);
    });
}

bool PiperTrajectoryBridge::group_reserved_locked(Group group) const
{
  return group == Group::kArm ? arm_reserved_ : gripper_reserved_;
}

void PiperTrajectoryBridge::set_group_reserved_locked(
  Group group,
  bool value)
{
  if (group == Group::kArm) {
    arm_reserved_ = value;
  } else {
    gripper_reserved_ = value;
  }
}

bool PiperTrajectoryBridge::joint_is_active_locked(
  const std::string & name) const
{
  if (name == "joint7") {
    return gripper_reserved_;
  }
  return arm_reserved_ &&
         std::find(arm_joints().begin(), arm_joints().end(), name) !=
         arm_joints().end();
}

}  // namespace piper_trajectory_bridge
