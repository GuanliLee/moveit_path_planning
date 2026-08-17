#include "path_planning_server/path_planning_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <stdexcept>
#include <utility>
#include <vector>

#include <boost/variant/get.hpp>
#include <Eigen/Core>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/duration.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace path_planning_server
{

namespace
{

constexpr char kLeftTrajectoryAction[] =
  "/left/arm_controller/follow_joint_trajectory";
constexpr char kRightTrajectoryAction[] =
  "/right/arm_controller/follow_joint_trajectory";
class ActiveRequestGuard
{
public:
  ActiveRequestGuard(std::atomic<bool> & active, std::atomic<bool> & canceled)
  : active_(active), canceled_(canceled)
  {
    active_.store(true);
  }
  ~ActiveRequestGuard()
  {
    active_.store(false);
    canceled_.store(false);
  }
private:
  std::atomic<bool> & active_;
  std::atomic<bool> & canceled_;
};

constexpr char kLeftGripperAction[] =
  "/left/gripper_controller/follow_joint_trajectory";
constexpr char kRightGripperAction[] =
  "/right/gripper_controller/follow_joint_trajectory";
constexpr double kMaxGripperOpening = 0.1;
constexpr double kIkCheckTimeout = 0.05;
constexpr double kJointTargetTolerance = 1.0e-3;
constexpr std::chrono::seconds kActionServerWaitTimeout{5};
constexpr std::chrono::seconds kPlanningSceneServiceWaitTimeout{2};
using SteadyClock = std::chrono::steady_clock;

double elapsedMs(const SteadyClock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

std::size_t ensureAcmEntry(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & name)
{
  const auto existing =
    std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
  if (existing != acm.entry_names.end()) {
    const std::size_t size = acm.entry_names.size();
    for (auto & entry : acm.entry_values) {
      entry.enabled.resize(size, false);
    }
    return static_cast<std::size_t>(
      std::distance(acm.entry_names.begin(), existing));
  }

  const std::size_t index = acm.entry_names.size();
  acm.entry_names.push_back(name);
  for (auto & entry : acm.entry_values) {
    entry.enabled.resize(index + 1, false);
  }
  moveit_msgs::msg::AllowedCollisionEntry entry;
  entry.enabled.assign(index + 1, false);
  acm.entry_values.push_back(entry);
  return index;
}

bool hasAcmEntry(
  const moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & name)
{
  return std::find(acm.entry_names.begin(), acm.entry_names.end(), name) !=
         acm.entry_names.end();
}

void setAcmPair(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & first,
  const std::string & second,
  const bool allowed,
  const bool add_missing)
{
  if (!add_missing && (!hasAcmEntry(acm, first) || !hasAcmEntry(acm, second))) {
    return;
  }

  const std::size_t first_index = ensureAcmEntry(acm, first);
  const std::size_t second_index = ensureAcmEntry(acm, second);
  const std::size_t size = acm.entry_names.size();
  for (auto & entry : acm.entry_values) {
    entry.enabled.resize(size, false);
  }
  acm.entry_values[first_index].enabled[second_index] = allowed;
  acm.entry_values[second_index].enabled[first_index] = allowed;
}

}  // namespace

PathPlanningNode::PathPlanningNode(const rclcpp::NodeOptions & options)
: Node("path_planning_server", options)
{
  const auto string_parameter =
    [this](const std::string & name, const std::string & default_value) {
      if (!has_parameter(name)) {
        return declare_parameter<std::string>(name, default_value);
      }
      return get_parameter(name).as_string();
    };
  const auto double_parameter =
    [this](const std::string & name, const double default_value) {
      if (!has_parameter(name)) {
        return declare_parameter<double>(name, default_value);
      }
      return get_parameter(name).as_double();
    };
  const auto integer_parameter =
    [this](const std::string & name, const std::int64_t default_value) {
      if (!has_parameter(name)) {
        return declare_parameter<std::int64_t>(name, default_value);
      }
      return get_parameter(name).as_int();
    };
  const auto bool_parameter =
    [this](const std::string & name, const bool default_value) {
      if (!has_parameter(name)) {
        return declare_parameter<bool>(name, default_value);
      }
      return get_parameter(name).as_bool();
    };

  world_frame_ = string_parameter("world_frame", "world");
  left_planning_group_ = string_parameter("left_planning_group", "left_arm");
  right_planning_group_ = string_parameter("right_planning_group", "right_arm");
  left_end_effector_link_ =
    string_parameter("left_end_effector_link", "left_link6");
  right_end_effector_link_ =
    string_parameter("right_end_effector_link", "right_link6");
  scene_index_file_ =
    string_parameter("scene_index_file", "config/scenes/scene_index.yaml");
  default_scene_id_ =
    static_cast<std::uint32_t>(integer_parameter("default_scene_id", 1));
  planning_time_ = double_parameter("planning_time", 5.0);
  planning_attempts_ =
    static_cast<unsigned int>(integer_parameter("planning_attempts", 1));
  velocity_scaling_ = double_parameter("velocity_scaling", 0.1);
  acceleration_scaling_ = double_parameter("acceleration_scaling", 0.1);
  position_tolerance_ = double_parameter("position_tolerance", 0.005);
  orientation_tolerance_ = double_parameter("orientation_tolerance", 0.05);
  gripper_move_duration_ = double_parameter("gripper_move_duration", 1.0);
  gripper_release_duration_ = double_parameter("gripper_release_duration", 3.0);
  if (!std::isfinite(gripper_move_duration_) || gripper_move_duration_ <= 0.0) {
    throw std::invalid_argument("gripper_move_duration must be finite and positive");
  }
  if (!std::isfinite(gripper_release_duration_) || gripper_release_duration_ <= 0.0) {
    throw std::invalid_argument("gripper_release_duration must be finite and positive");
  }
  gripper_effort_ = double_parameter("gripper_effort", 4.0);
  if (!std::isfinite(gripper_effort_) || gripper_effort_ < 0.0 || gripper_effort_ > 5.0) {
    throw std::invalid_argument("gripper_effort must be finite and in [0.0, 5.0]");
  }
  gripper_contact_opening_threshold_ =
    double_parameter("gripper_contact_opening_threshold", 0.02);
  gripper_contact_goal_tolerance_ =
    double_parameter("gripper_contact_goal_tolerance", 0.04);
  gripper_contact_goal_time_tolerance_ =
    double_parameter("gripper_contact_goal_time_tolerance", 0.3);
  if (!std::isfinite(gripper_contact_goal_time_tolerance_) ||
    gripper_contact_goal_time_tolerance_ < 0.0)
  {
    throw std::invalid_argument("gripper_contact_goal_time_tolerance must be finite and non-negative");
  }
  treat_gripper_contact_abort_as_success_ =
    bool_parameter("treat_gripper_contact_abort_as_success", true);
  const std::string grasp_ellipsoid_mesh_resource = string_parameter(
    "grasp_ellipsoid.mesh_resource",
    "package://path_planning_server/meshes/grasp_ellipsoid/unit_sphere.stl");
  const double grasp_ellipsoid_radius_x =
    double_parameter("grasp_ellipsoid.radius_x_m", 0.06);
  const double grasp_ellipsoid_radius_y =
    double_parameter("grasp_ellipsoid.radius_y_m", 0.06);
  const double grasp_ellipsoid_radius_z =
    double_parameter("grasp_ellipsoid.radius_z_m", 0.12);
  grasp_ellipsoid_center_z_ =
    double_parameter("grasp_ellipsoid.center_z_m", 0.1358);
  execute_trajectory_ = bool_parameter("execute_trajectory", false);

  std::unique_ptr<shapes::Mesh> grasp_ellipsoid_shape(
    shapes::createMeshFromResource(
      grasp_ellipsoid_mesh_resource,
      Eigen::Vector3d(
        grasp_ellipsoid_radius_x,
        grasp_ellipsoid_radius_y,
        grasp_ellipsoid_radius_z)));
  if (!grasp_ellipsoid_shape) {
    throw std::runtime_error(
      "Failed to load grasp ellipsoid mesh: " + grasp_ellipsoid_mesh_resource);
  }
  shapes::ShapeMsg grasp_ellipsoid_message;
  if (!shapes::constructMsgFromShape(
      grasp_ellipsoid_shape.get(), grasp_ellipsoid_message))
  {
    throw std::runtime_error("Failed to convert grasp ellipsoid mesh");
  }
  const auto * grasp_ellipsoid_mesh =
    boost::get<shape_msgs::msg::Mesh>(&grasp_ellipsoid_message);
  if (!grasp_ellipsoid_mesh) {
    throw std::runtime_error("Grasp ellipsoid resource is not a mesh");
  }
  grasp_ellipsoid_mesh_ = *grasp_ellipsoid_mesh;
}

void PathPlanningNode::initialize()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), true);

  left_move_group_ = std::make_unique<MoveGroupInterface>(
    shared_from_this(), left_planning_group_, tf_buffer_);
  right_move_group_ = std::make_unique<MoveGroupInterface>(
    shared_from_this(), right_planning_group_, tf_buffer_);
  configureMoveGroup(*left_move_group_, left_end_effector_link_);
  configureMoveGroup(*right_move_group_, right_end_effector_link_);

  planning_scene_ =
    std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  get_planning_scene_client_ = create_client<GetPlanningScene>("/get_planning_scene");
  static_scene_loader_ =
    std::make_unique<StaticSceneLoader>(scene_index_file_, planning_scene_);

  cancel_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::string scene_message;
  if (!static_scene_loader_->loadScene(default_scene_id_, scene_message)) {
    throw std::runtime_error(scene_message);
  }

  cancel_service_ = create_service<Trigger>(
    "/cancel_plan_execution",
    std::bind(
      &PathPlanningNode::handleCancelRequest, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cancel_callback_group_);

  action_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  left_trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    shared_from_this(), kLeftTrajectoryAction, action_callback_group_);
  right_trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    shared_from_this(), kRightTrajectoryAction, action_callback_group_);
  left_gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    shared_from_this(), kLeftGripperAction, action_callback_group_);
  right_gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    shared_from_this(), kRightGripperAction, action_callback_group_);

  service_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_ = create_service<PlanToPose>(
    "/plan_to_pose",
    std::bind(
      &PathPlanningNode::handlePlanRequest, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    service_callback_group_);
  joint_service_ = create_service<PlanToJoints>(
    "/plan_to_joints",
    std::bind(
      &PathPlanningNode::handleJointPlanRequest, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    service_callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "Ready: /plan_to_pose, /plan_to_joints, and /cancel_plan_execution, "
    "groups [%s, %s], scene %u, execute trajectory: %s",
    left_planning_group_.c_str(), right_planning_group_.c_str(), default_scene_id_,
    execute_trajectory_ ? "true" : "false");
}

void PathPlanningNode::configureMoveGroup(
  MoveGroupInterface & move_group,
  const std::string & end_effector_link)
{
  move_group.setPoseReferenceFrame(world_frame_);
  if (!move_group.setEndEffectorLink(end_effector_link)) {
    throw std::runtime_error("End effector link not found: " + end_effector_link);
  }
  move_group.setPlanningTime(planning_time_);
  move_group.setNumPlanningAttempts(planning_attempts_);
  move_group.setMaxVelocityScalingFactor(velocity_scaling_);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);
  move_group.setGoalPositionTolerance(position_tolerance_);
  move_group.setGoalOrientationTolerance(orientation_tolerance_);
}

void PathPlanningNode::handleCancelRequest(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  cancel_requested_.store(true);
  try {
    for (const auto & client : {
        left_trajectory_client_, right_trajectory_client_,
        left_gripper_client_, right_gripper_client_})
    {
      if (client) {
        client->async_cancel_all_goals();
      }
    }
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("failed to request action cancellation: ") + error.what();
    return;
  }
  response->success = true;
  response->message = request_active_.load() ? "cancel requested for active planning/execution" : "cancel queued for pending planning request";
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

void PathPlanningNode::handlePlanRequest(
  const std::shared_ptr<PlanToPose::Request> request,
  std::shared_ptr<PlanToPose::Response> response)
{
  const auto request_start = SteadyClock::now();
  response->success = false;
  if (cancel_requested_.exchange(false)) {
    response->message = "Execution canceled before planning started";
    return;
  }
  ActiveRequestGuard active_request(request_active_, cancel_requested_);


  MoveGroupInterface * move_group = nullptr;
  std::string planning_group;
  std::string end_effector_link;
  if (request->arm_name == "left") {
    move_group = left_move_group_.get();
    planning_group = left_planning_group_;
    end_effector_link = left_end_effector_link_;
  } else if (request->arm_name == "right") {
    move_group = right_move_group_.get();
    planning_group = right_planning_group_;
    end_effector_link = right_end_effector_link_;
  } else {
    response->message = "Invalid arm name";
    return;
  }

  if (!static_scene_loader_->hasScene(request->scene_id)) {
    response->message = "Scene ID not found";
    return;
  }
  if (request->target_pose.header.frame_id.empty()) {
    response->message = "Target frame is empty";
    return;
  }

  const auto & pose = request->target_pose.pose;
  const std::array<double, 7> values = {
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
  };
  if (!std::all_of(values.begin(), values.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    response->message = "Target pose contains non-finite values";
    return;
  }

  const double quaternion_norm_squared =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  if (quaternion_norm_squared == 0.0) {
    response->message = "Target orientation is zero";
    return;
  }
  if (request->gripper_command &&
    (!std::isfinite(request->gripper_opening_m) ||
    request->gripper_opening_m < 0.0 ||
    request->gripper_opening_m > kMaxGripperOpening))
  {
    response->message = "Gripper opening must be finite and in [0.0, 0.1] meters";
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=request_start arm=%s scene=%u gripper=%s "
    "opening=%.4f keep_ellipsoid=%s frame=%s",
    request->arm_name.c_str(), static_cast<unsigned int>(request->scene_id),
    request->gripper_command ? "true" : "false", request->gripper_opening_m,
    request->keep_grasp_ellipsoid ? "true" : "false",
    request->target_pose.header.frame_id.c_str());

  std::string scene_message;
  auto phase_start = SteadyClock::now();
  if (!static_scene_loader_->loadScene(request->scene_id, scene_message)) {
    response->message = scene_message;
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=load_scene success=false elapsed_ms=%.1f total_ms=%.1f "
      "message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=load_scene success=true elapsed_ms=%.1f",
    elapsedMs(phase_start));

  if (!request->keep_grasp_ellipsoid) {
    std::string ellipsoid_message;
    phase_start = SteadyClock::now();
    if (!setGraspEllipsoid(request->arm_name, false, ellipsoid_message)) {
      response->message = "Failed to clear grasp ellipsoid before planning: " + ellipsoid_message;
      RCLCPP_INFO(
        get_logger(),
        "[TIMING] plan_to_pose phase=clear_grasp_ellipsoid_before_plan success=false "
        "elapsed_ms=%.1f total_ms=%.1f message=%s",
        elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=clear_grasp_ellipsoid_before_plan success=true elapsed_ms=%.1f",
      elapsedMs(phase_start));
  }

  geometry_msgs::msg::PoseStamped target_pose;
  phase_start = SteadyClock::now();
  if (!transformTarget(request->target_pose, target_pose, response->message)) {
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=transform_target success=false elapsed_ms=%.1f "
      "total_ms=%.1f message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=transform_target success=true elapsed_ms=%.1f",
    elapsedMs(phase_start));
  auto & orientation = target_pose.pose.orientation;
  const double orientation_norm = std::sqrt(
    orientation.x * orientation.x +
    orientation.y * orientation.y +
    orientation.z * orientation.z +
    orientation.w * orientation.w);
  orientation.x /= orientation_norm;
  orientation.y /= orientation_norm;
  orientation.z /= orientation_norm;
  orientation.w /= orientation_norm;

  phase_start = SteadyClock::now();
  auto current_state = move_group->getCurrentState(1.0);
  if (!current_state) {
    response->message = "Current robot state unavailable";
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=get_current_state success=false elapsed_ms=%.1f "
      "total_ms=%.1f message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=get_current_state success=true elapsed_ms=%.1f",
    elapsedMs(phase_start));

  phase_start = SteadyClock::now();
  if (!validateTargetPose(
      *move_group, *current_state, planning_group, end_effector_link,
      target_pose, request->keep_grasp_ellipsoid, response->message))
  {
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=validate_target success=false elapsed_ms=%.1f "
      "total_ms=%.1f message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=validate_target success=true elapsed_ms=%.1f",
    elapsedMs(phase_start));

  move_group->clearPoseTargets();
  // CurrentStateMonitor only contains joints and TF. Serializing that state as
  // a complete start state would clear attached collision objects, including
  // the grasp ellipsoid, from this planning request. Use an empty diff start
  // state so MoveIt retains attached objects from its live PlanningScene.
  move_group->setStartStateToCurrentState();
  if (!move_group->setPoseTarget(target_pose, end_effector_link)) {
    response->message = "Failed to set pose target";
    return;
  }

  MoveGroupInterface::Plan plan;
  phase_start = SteadyClock::now();
  const auto result = move_group->plan(plan);
  move_group->clearPoseTargets();

  if (cancel_requested_.load()) {
    response->message = "Execution canceled during planning";
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=planning canceled=true elapsed_ms=%.1f total_ms=%.1f",
      elapsedMs(phase_start), elapsedMs(request_start));
    return;
  }
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    response->message = "Planning failed";
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=planning success=false elapsed_ms=%.1f total_ms=%.1f "
      "message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=planning success=true elapsed_ms=%.1f points=%zu",
    elapsedMs(phase_start), plan.trajectory.joint_trajectory.points.size());
  if (plan.trajectory.joint_trajectory.points.empty()) {
    response->message = "Planned trajectory is empty";
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=planning success=false elapsed_ms=%.1f total_ms=%.1f "
      "message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }

  response->trajectory = plan.trajectory;
  if (!execute_trajectory_) {
    response->success = true;
    response->message = "Planning succeeded";
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=request_finish success=true total_ms=%.1f message=%s",
      elapsedMs(request_start), response->message.c_str());
    return;
  }

  if (cancel_requested_.load()) {
    response->message = "Execution canceled before trajectory start";
    return;
  }
  phase_start = SteadyClock::now();
  if (!executeTrajectory(
      request->arm_name, response->trajectory, response->message))
  {
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=execute_trajectory success=false elapsed_ms=%.1f "
      "total_ms=%.1f message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=execute_trajectory success=true elapsed_ms=%.1f",
    elapsedMs(phase_start));

  if (cancel_requested_.load()) {
    response->message = "Execution canceled after trajectory";
    return;
  }
  const bool release_grasp_ellipsoid_after_gripper =
    request->keep_grasp_ellipsoid &&
    request->gripper_command &&
    request->gripper_opening_m > gripper_contact_opening_threshold_;

  if (request->gripper_command) {
    std::string gripper_message;
    const double gripper_duration = release_grasp_ellipsoid_after_gripper ?
      gripper_release_duration_ : gripper_move_duration_;
    phase_start = SteadyClock::now();
    if (!executeGripperCommand(
        request->arm_name, request->gripper_opening_m, gripper_duration, gripper_message))
    {
      response->message =
        "Arm trajectory execution succeeded, but " + gripper_message;
      RCLCPP_INFO(
        get_logger(),
        "[TIMING] plan_to_pose phase=execute_gripper success=false elapsed_ms=%.1f "
        "total_ms=%.1f message=%s",
        elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=execute_gripper success=true opening=%.4f "
      "duration_s=%.2f elapsed_ms=%.1f",
      request->gripper_opening_m, gripper_duration, elapsedMs(phase_start));
  }

  if (cancel_requested_.load()) {
    response->message = "Execution canceled after gripper command";
    return;
  }
  std::string ellipsoid_message;
  const bool final_grasp_ellipsoid_enabled =
    request->keep_grasp_ellipsoid && !release_grasp_ellipsoid_after_gripper;
  phase_start = SteadyClock::now();
  if (!setGraspEllipsoid(
      request->arm_name, final_grasp_ellipsoid_enabled, ellipsoid_message))
  {
    response->message = request->gripper_command ?
      "Arm trajectory and gripper execution succeeded, but " + ellipsoid_message :
      "Arm trajectory execution succeeded, but " + ellipsoid_message;
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_pose phase=update_grasp_ellipsoid success=false enabled=%s "
      "elapsed_ms=%.1f total_ms=%.1f message=%s",
      final_grasp_ellipsoid_enabled ? "true" : "false",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=update_grasp_ellipsoid success=true enabled=%s elapsed_ms=%.1f",
    final_grasp_ellipsoid_enabled ? "true" : "false", elapsedMs(phase_start));

  response->success = true;
  response->message = request->gripper_command ?
    "Planning, trajectory, gripper, and grasp ellipsoid update succeeded" :
    "Planning, trajectory, and grasp ellipsoid update succeeded";
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_pose phase=request_finish success=true total_ms=%.1f message=%s",
    elapsedMs(request_start), response->message.c_str());
}

void PathPlanningNode::handleJointPlanRequest(
  const std::shared_ptr<PlanToJoints::Request> request,
  std::shared_ptr<PlanToJoints::Response> response)
{
  const auto request_start = SteadyClock::now();
  response->success = false;
  if (cancel_requested_.exchange(false)) {
    response->message = "Execution canceled before planning started";
    return;
  }
  ActiveRequestGuard active_request(request_active_, cancel_requested_);

  MoveGroupInterface * move_group = nullptr;
  std::string planning_group;
  if (request->arm_name == "left") {
    move_group = left_move_group_.get();
    planning_group = left_planning_group_;
  } else if (request->arm_name == "right") {
    move_group = right_move_group_.get();
    planning_group = right_planning_group_;
  } else {
    response->message = "Invalid arm name";
    return;
  }

  if (!static_scene_loader_->hasScene(request->scene_id)) {
    response->message = "Scene ID not found";
    return;
  }
  if (!std::all_of(
      request->joint_positions.begin(), request->joint_positions.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    response->message = "Joint target contains non-finite values";
    return;
  }
  const std::vector<double> target_joints(
    request->joint_positions.begin(), request->joint_positions.end());
  if (request->gripper_command &&
    (!std::isfinite(request->gripper_opening_m) ||
    request->gripper_opening_m < 0.0 ||
    request->gripper_opening_m > kMaxGripperOpening))
  {
    response->message = "Gripper opening must be finite and in [0.0, 0.1] meters";
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_joints phase=request_start arm=%s scene=%u "
    "gripper=%s opening=%.4f keep_ellipsoid=%s "
    "joints=[%.9f,%.9f,%.9f,%.9f,%.9f,%.9f]",
    request->arm_name.c_str(), static_cast<unsigned int>(request->scene_id),
    request->gripper_command ? "true" : "false", request->gripper_opening_m,
    request->keep_grasp_ellipsoid ? "true" : "false",
    target_joints[0], target_joints[1], target_joints[2],
    target_joints[3], target_joints[4], target_joints[5]);

  std::string scene_message;
  auto phase_start = SteadyClock::now();
  if (!static_scene_loader_->loadScene(request->scene_id, scene_message)) {
    response->message = scene_message;
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_joints phase=load_scene success=true elapsed_ms=%.1f",
    elapsedMs(phase_start));

  if (!request->keep_grasp_ellipsoid) {
    std::string ellipsoid_message;
    phase_start = SteadyClock::now();
    if (!setGraspEllipsoid(request->arm_name, false, ellipsoid_message)) {
      response->message =
        "Failed to clear grasp ellipsoid before planning: " + ellipsoid_message;
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_joints phase=clear_grasp_ellipsoid_before_plan "
      "success=true elapsed_ms=%.1f",
      elapsedMs(phase_start));
  }

  phase_start = SteadyClock::now();
  auto current_state = move_group->getCurrentState(1.0);
  if (!current_state) {
    response->message = "Current robot state unavailable";
    return;
  }
  const auto * joint_group = current_state->getJointModelGroup(planning_group);
  if (!joint_group || joint_group->getVariableCount() != target_joints.size()) {
    response->message = "Planning group does not contain exactly six joints";
    return;
  }
  std::vector<double> current_joints;
  current_state->copyJointGroupPositions(joint_group, current_joints);
  double max_joint_error = 0.0;
  for (std::size_t index = 0; index < target_joints.size(); ++index) {
    max_joint_error = std::max(
      max_joint_error, std::abs(target_joints[index] - current_joints[index]));
  }
  const bool arm_motion_required = max_joint_error > kJointTargetTolerance;

  moveit::core::RobotState target_state(*current_state);
  target_state.setJointGroupPositions(joint_group, target_joints);
  target_state.update();
  if (!target_state.satisfiesBounds(joint_group)) {
    response->message = "Joint target violates model bounds";
    return;
  }

  planning_scene::PlanningScene collision_scene(move_group->getRobotModel());
  collision_scene.getTransformsNonConst().setTransform(
    Eigen::Isometry3d::Identity(), world_frame_);
  collision_scene.setCurrentState(*current_state);
  for (const auto & [id, object] : planning_scene_->getObjects()) {
    (void)id;
    collision_scene.processCollisionObjectMsg(object);
  }
  for (const auto & [id, object] : planning_scene_->getAttachedObjects()) {
    (void)id;
    collision_scene.processAttachedCollisionObjectMsg(object);
  }
  if (collision_scene.isStateColliding(target_state, planning_group)) {
    response->message = "Joint target is in collision";
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_joints phase=validate_target success=true elapsed_ms=%.1f",
    elapsedMs(phase_start));

  const auto finish_execution = [&](const std::string & arm_success_message) -> bool {
      if (cancel_requested_.load()) {
        response->message = "Execution canceled after arm motion";
        return false;
      }
      const bool release_grasp_ellipsoid_after_gripper =
        request->keep_grasp_ellipsoid &&
        request->gripper_command &&
        request->gripper_opening_m > gripper_contact_opening_threshold_;

      if (request->gripper_command) {
        std::string gripper_message;
        const double gripper_duration = release_grasp_ellipsoid_after_gripper ?
          gripper_release_duration_ : gripper_move_duration_;
        phase_start = SteadyClock::now();
        if (!executeGripperCommand(
            request->arm_name, request->gripper_opening_m, gripper_duration, gripper_message))
        {
          response->message = arm_success_message + ", but " + gripper_message;
          RCLCPP_INFO(
            get_logger(),
            "[TIMING] plan_to_joints phase=execute_gripper success=false "
            "elapsed_ms=%.1f total_ms=%.1f message=%s",
            elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
          return false;
        }
        RCLCPP_INFO(
          get_logger(),
          "[TIMING] plan_to_joints phase=execute_gripper success=true opening=%.4f "
          "duration_s=%.2f elapsed_ms=%.1f",
          request->gripper_opening_m, gripper_duration, elapsedMs(phase_start));
      }

      if (cancel_requested_.load()) {
        response->message = "Execution canceled after gripper command";
        return false;
      }
      std::string ellipsoid_message;
      const bool final_grasp_ellipsoid_enabled =
        request->keep_grasp_ellipsoid && !release_grasp_ellipsoid_after_gripper;
      phase_start = SteadyClock::now();
      if (!setGraspEllipsoid(
          request->arm_name, final_grasp_ellipsoid_enabled, ellipsoid_message))
      {
        response->message = arm_success_message +
          (request->gripper_command ? ", gripper execution succeeded, but " : ", but ") +
          ellipsoid_message;
        RCLCPP_INFO(
          get_logger(),
          "[TIMING] plan_to_joints phase=update_grasp_ellipsoid success=false enabled=%s "
          "elapsed_ms=%.1f total_ms=%.1f message=%s",
          final_grasp_ellipsoid_enabled ? "true" : "false",
          elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
        return false;
      }

      response->success = true;
      response->message = arm_success_message;
      if (request->gripper_command) {
        response->message += ", gripper execution succeeded";
      }
      response->message += ", grasp ellipsoid update succeeded";
      RCLCPP_INFO(
        get_logger(),
        "[TIMING] plan_to_joints phase=request_finish success=true total_ms=%.1f "
        "message=%s",
        elapsedMs(request_start), response->message.c_str());
      return true;
    };

  if (!arm_motion_required) {
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_joints phase=arm_motion skipped=true max_joint_error=%.9f "
      "tolerance=%.9f",
      max_joint_error, kJointTargetTolerance);
    if (!execute_trajectory_) {
      response->success = true;
      response->message = "Joint target already reached";
      return;
    }
    (void)finish_execution("Joint target already reached");
    return;
  }

  move_group->clearPoseTargets();
  move_group->setStartStateToCurrentState();
  if (!move_group->setJointValueTarget(target_joints)) {
    response->message = "Failed to set joint target";
    return;
  }

  MoveGroupInterface::Plan plan;
  phase_start = SteadyClock::now();
  const auto result = move_group->plan(plan);
  if (cancel_requested_.load()) {
    response->message = "Execution canceled during planning";
    return;
  }
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    response->message = "Joint planning failed";
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_joints phase=planning success=false elapsed_ms=%.1f "
      "total_ms=%.1f",
      elapsedMs(phase_start), elapsedMs(request_start));
    return;
  }
  if (plan.trajectory.joint_trajectory.points.empty()) {
    response->message = "Planned joint trajectory is empty";
    return;
  }
  response->trajectory = plan.trajectory;
  RCLCPP_INFO(
    get_logger(),
    "[TIMING] plan_to_joints phase=planning success=true elapsed_ms=%.1f points=%zu",
    elapsedMs(phase_start), plan.trajectory.joint_trajectory.points.size());

  if (!execute_trajectory_) {
    response->success = true;
    response->message = "Joint planning succeeded";
    return;
  }
  if (cancel_requested_.load()) {
    response->message = "Execution canceled before trajectory start";
    return;
  }
  phase_start = SteadyClock::now();
  if (!executeTrajectory(
      request->arm_name, response->trajectory, response->message))
  {
    RCLCPP_INFO(
      get_logger(),
      "[TIMING] plan_to_joints phase=execute_trajectory success=false "
      "elapsed_ms=%.1f total_ms=%.1f message=%s",
      elapsedMs(phase_start), elapsedMs(request_start), response->message.c_str());
    return;
  }

  (void)finish_execution("Joint planning and trajectory execution succeeded");
}

bool PathPlanningNode::validateTargetPose(
  MoveGroupInterface & move_group,
  const moveit::core::RobotState & current_state,
  const std::string & planning_group,
  const std::string & end_effector_link,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const bool keep_grasp_ellipsoid,
  std::string & message) const
{
  planning_scene::PlanningScene collision_scene(move_group.getRobotModel());
  collision_scene.getTransformsNonConst().setTransform(
    Eigen::Isometry3d::Identity(), world_frame_);
  collision_scene.setCurrentState(current_state);

  for (const auto & [id, object] : planning_scene_->getObjects()) {
    (void)id;
    collision_scene.processCollisionObjectMsg(object);
  }
  for (const auto & [id, object] : planning_scene_->getAttachedObjects()) {
    (void)id;
    collision_scene.processAttachedCollisionObjectMsg(object);
  }

  moveit::core::RobotState target_state(collision_scene.getCurrentState());
  const auto * joint_group = target_state.getJointModelGroup(planning_group);
  if (!target_state.setFromIK(
      joint_group, target_pose.pose, end_effector_link, kIkCheckTimeout))
  {
    message = "Target pose has no IK solution";
    return false;
  }

  const std::string ellipsoid_id =
    planning_group == left_planning_group_ ?
    "left_grasp_ellipsoid" : "right_grasp_ellipsoid";
  target_state.clearAttachedBody(ellipsoid_id);

  if (keep_grasp_ellipsoid) {
    shapes::ShapeConstPtr ellipsoid_shape(
      shapes::constructShapeFromMsg(grasp_ellipsoid_mesh_));
    Eigen::Isometry3d ellipsoid_pose = Eigen::Isometry3d::Identity();
    ellipsoid_pose.translation().z() = grasp_ellipsoid_center_z_;
    const std::vector<shapes::ShapeConstPtr> ellipsoid_shapes = {ellipsoid_shape};
    const EigenSTL::vector_Isometry3d ellipsoid_shape_poses = {
      Eigen::Isometry3d::Identity()};
    const std::vector<std::string> touch_links = {
      end_effector_link,
      planning_group == left_planning_group_ ? "left_gripper_base" : "right_gripper_base",
      planning_group == left_planning_group_ ? "left_link7" : "right_link7",
      planning_group == left_planning_group_ ? "left_link8" : "right_link8"
    };
    target_state.attachBody(
      ellipsoid_id,
      ellipsoid_pose,
      ellipsoid_shapes,
      ellipsoid_shape_poses,
      touch_links,
      end_effector_link);
  }
  target_state.update();

  if (collision_scene.isStateColliding(target_state, planning_group)) {
    message = "Target pose is in collision";
    return false;
  }

  return true;
}

bool PathPlanningNode::setGraspEllipsoid(
  const std::string & arm_name,
  const bool enabled,
  std::string & message)
{
  const std::string link_name =
    arm_name == "left" ? left_end_effector_link_ : right_end_effector_link_;
  const std::string link_prefix = arm_name + "_";
  const std::string ellipsoid_id = arm_name + "_grasp_ellipsoid";

  moveit_msgs::msg::AttachedCollisionObject ellipsoid;
  ellipsoid.link_name = link_name;
  ellipsoid.object.header.frame_id = link_name;
  ellipsoid.object.id = ellipsoid_id;

  if (enabled) {
    ellipsoid.touch_links = {
      link_name,
      link_prefix + "gripper_base",
      link_prefix + "link7",
      link_prefix + "link8"
    };
    ellipsoid.object.meshes.push_back(grasp_ellipsoid_mesh_);

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.z = grasp_ellipsoid_center_z_;
    ellipsoid.object.mesh_poses.push_back(pose);
    ellipsoid.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  } else {
    ellipsoid.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  }

  if (enabled && !updateGraspEllipsoidAllowedCollisions(arm_name, true, message)) {
    return false;
  }

  if (!planning_scene_->applyAttachedCollisionObject(ellipsoid)) {
    message = enabled ?
      "failed to attach grasp ellipsoid" :
      "failed to remove grasp ellipsoid";
    return false;
  }

  if (!enabled) {
    const auto known_objects = planning_scene_->getKnownObjectNames();
    if (std::find(known_objects.begin(), known_objects.end(), ellipsoid_id) !=
      known_objects.end())
    {
      moveit_msgs::msg::CollisionObject world_object;
      world_object.header.frame_id = world_frame_;
      world_object.id = ellipsoid_id;
      world_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!planning_scene_->applyCollisionObject(world_object)) {
        message = "failed to remove grasp ellipsoid world object";
        return false;
      }
    }
  }

  if (!enabled && !updateGraspEllipsoidAllowedCollisions(arm_name, false, message)) {
    return false;
  }

  message = enabled ?
    "grasp ellipsoid attached" :
    "grasp ellipsoid removed";
  return true;
}

bool PathPlanningNode::updateGraspEllipsoidAllowedCollisions(
  const std::string & arm_name,
  const bool allowed,
  std::string & message)
{
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  if (!fetchAllowedCollisionMatrix(acm, message)) {
    return false;
  }

  const std::string ellipsoid_id = arm_name + "_grasp_ellipsoid";
  const std::string link_name =
    arm_name == "left" ? left_end_effector_link_ : right_end_effector_link_;
  const std::string link_prefix = arm_name + "_";
  const std::vector<std::string> touch_links = {
    link_name,
    link_prefix + "gripper_base",
    link_prefix + "link7",
    link_prefix + "link8"
  };

  for (const auto & touch_link : touch_links) {
    setAcmPair(acm, ellipsoid_id, touch_link, allowed, allowed);
  }

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.allowed_collision_matrix = acm;
  if (!planning_scene_->applyPlanningScene(scene)) {
    message = allowed ?
      "failed to allow grasp ellipsoid touch links" :
      "failed to clear grasp ellipsoid touch link allowances";
    return false;
  }

  message = allowed ?
    "grasp ellipsoid touch links allowed" :
    "grasp ellipsoid touch link allowances cleared";
  return true;
}

bool PathPlanningNode::fetchAllowedCollisionMatrix(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  std::string & message)
{
  if (!get_planning_scene_client_->wait_for_service(kPlanningSceneServiceWaitTimeout)) {
    message = "get planning scene service unavailable";
    return false;
  }

  auto request = std::make_shared<GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = get_planning_scene_client_->async_send_request(request);
  if (future.wait_for(kPlanningSceneServiceWaitTimeout) != std::future_status::ready) {
    message = "timeout fetching allowed collision matrix";
    return false;
  }

  const auto response = future.get();
  if (!response) {
    message = "get planning scene returned no response";
    return false;
  }
  acm = response->scene.allowed_collision_matrix;
  if (acm.entry_values.size() < acm.entry_names.size()) {
    acm.entry_values.resize(acm.entry_names.size());
  }
  for (auto & entry : acm.entry_values) {
    entry.enabled.resize(acm.entry_names.size(), false);
  }
  return true;
}

bool PathPlanningNode::transformTarget(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output,
  std::string & message) const
{
  if (input.header.frame_id == world_frame_) {
    output = input;
    return true;
  }

  try {
    output = tf_buffer_->transform(input, world_frame_, tf2::durationFromSec(1.0));
    return true;
  } catch (const tf2::TransformException & error) {
    message = std::string("Transform failed: ") + error.what();
    return false;
  }
}

bool PathPlanningNode::executeTrajectory(
  const std::string & arm_name,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & message)
{
  const auto action_client =
    arm_name == "left" ? left_trajectory_client_ : right_trajectory_client_;
  const std::string joint_prefix = arm_name + "_";

  if (!action_client->wait_for_action_server(kActionServerWaitTimeout)) {
    message = "Action server unavailable";
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = trajectory.joint_trajectory;
  goal.multi_dof_trajectory = trajectory.multi_dof_joint_trajectory;
  for (auto & joint_name : goal.trajectory.joint_names) {
    if (joint_name.rfind(joint_prefix, 0) == 0) {
      joint_name.erase(0, joint_prefix.size());
    }
  }

  try {
    auto goal_future = action_client->async_send_goal(goal);
    goal_future.wait();
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      message = "Trajectory goal rejected";
      return false;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    bool cancellation_sent = false;
    while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
      if (cancel_requested_.load() && !cancellation_sent) {
        action_client->async_cancel_goal(goal_handle);
        cancellation_sent = true;
      }
    }
    const auto wrapped_result = result_future.get();

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
        message = "Trajectory execution canceled";
      } else {
        message = "Trajectory execution aborted";
      }
      return false;
    }
    if (!wrapped_result.result) {
      message = "Trajectory execution aborted: Action returned no result";
      return false;
    }
    if (wrapped_result.result->error_code !=
      FollowJointTrajectory::Result::SUCCESSFUL)
    {
      message = "Trajectory execution aborted";
      if (!wrapped_result.result->error_string.empty()) {
        message += ": " + wrapped_result.result->error_string;
      }
      return false;
    }

    message = "Planning and trajectory execution succeeded";
    return true;
  } catch (const std::exception & error) {
    message = std::string("Trajectory execution aborted: ") + error.what();
    return false;
  }
}

bool PathPlanningNode::executeGripperCommand(
  const std::string & arm_name,
  double opening,
  double move_duration,
  std::string & message)
{
  const auto action_client =
    arm_name == "left" ? left_gripper_client_ : right_gripper_client_;

  if (!action_client->wait_for_action_server(kActionServerWaitTimeout)) {
    message = "gripper action server is unavailable";
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"joint7"};
  goal.trajectory.points.resize(1);
  goal.trajectory.points.front().positions = {opening / 2.0};
  goal.trajectory.points.front().effort = {gripper_effort_};
  goal.trajectory.points.front().time_from_start =
    rclcpp::Duration::from_seconds(move_duration);
  const bool contact_close_command =
    opening <= gripper_contact_opening_threshold_;
  if (contact_close_command) {
    goal.goal_time_tolerance =
      rclcpp::Duration::from_seconds(gripper_contact_goal_time_tolerance_);
  }
  if (contact_close_command && gripper_contact_goal_tolerance_ > 0.0) {
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint7";
    tolerance.position = gripper_contact_goal_tolerance_;
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    goal.goal_tolerance = {tolerance};
  }

  try {
    auto goal_future = action_client->async_send_goal(goal);
    goal_future.wait();
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      message = "gripper trajectory goal was rejected";
      return false;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    bool cancellation_sent = false;
    while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
      if (cancel_requested_.load() && !cancellation_sent) {
        action_client->async_cancel_goal(goal_handle);
        cancellation_sent = true;
      }
    }
    const auto wrapped_result = result_future.get();

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      if (contact_close_command && treat_gripper_contact_abort_as_success_ &&
        wrapped_result.code == rclcpp_action::ResultCode::ABORTED)
      {
        message = "gripper close reached contact; treating aborted goal tolerance as success";
        return true;
      }
      if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
        message = "gripper trajectory execution was canceled";
      } else {
        message = "gripper trajectory execution was aborted";
      }
      return false;
    }
    if (!wrapped_result.result) {
      message = "gripper trajectory execution returned no result";
      return false;
    }
    if (wrapped_result.result->error_code !=
      FollowJointTrajectory::Result::SUCCESSFUL)
    {
      if (contact_close_command && treat_gripper_contact_abort_as_success_ &&
        wrapped_result.result->error_code ==
        FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED)
      {
        message = "gripper close reached contact; treating goal tolerance violation as success";
        return true;
      }
      message = "gripper trajectory execution was aborted";
      if (!wrapped_result.result->error_string.empty()) {
        message += ": " + wrapped_result.result->error_string;
      }
      return false;
    }

    message = "gripper execution succeeded";
    return true;
  } catch (const std::exception & error) {
    message = std::string("gripper trajectory execution was aborted: ") +
      error.what();
    return false;
  }
}

}  // namespace path_planning_server
