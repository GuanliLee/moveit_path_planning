#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <path_planning_interfaces/srv/plan_to_joints.hpp>
#include <path_planning_interfaces/srv/plan_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "path_planning_server/static_scene_loader.hpp"

namespace path_planning_server
{

class PathPlanningNode : public rclcpp::Node
{
public:
  explicit PathPlanningNode(const rclcpp::NodeOptions & options);

  void initialize();

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FollowJointTrajectoryClient = rclcpp_action::Client<FollowJointTrajectory>;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using PlanToJoints = path_planning_interfaces::srv::PlanToJoints;
  using PlanToPose = path_planning_interfaces::srv::PlanToPose;
  using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
  using Trigger = std_srvs::srv::Trigger;

  void configureMoveGroup(
    MoveGroupInterface & move_group,
    const std::string & end_effector_link);
  void handlePlanRequest(
    const std::shared_ptr<PlanToPose::Request> request,
    std::shared_ptr<PlanToPose::Response> response);
  void handleJointPlanRequest(
    const std::shared_ptr<PlanToJoints::Request> request,
    std::shared_ptr<PlanToJoints::Response> response);
  void handleCancelRequest(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  bool transformTarget(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output,
    std::string & message) const;
  bool validateTargetPose(
    MoveGroupInterface & move_group,
    const moveit::core::RobotState & current_state,
    const std::string & planning_group,
    const std::string & end_effector_link,
    const geometry_msgs::msg::PoseStamped & target_pose,
    bool keep_grasp_ellipsoid,
    std::string & message) const;
  bool executeTrajectory(
    const std::string & arm_name,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & message);
  bool executeGripperCommand(
    const std::string & arm_name,
    double opening,
    double move_duration,
    std::string & message);
  bool setGraspEllipsoid(
    const std::string & arm_name,
    bool enabled,
    std::string & message);
  bool updateGraspEllipsoidAllowedCollisions(
    const std::string & arm_name,
    bool allowed,
    std::string & message);
  bool fetchAllowedCollisionMatrix(
    moveit_msgs::msg::AllowedCollisionMatrix & acm,
    std::string & message);

  std::string world_frame_;
  std::string left_planning_group_;
  std::string right_planning_group_;
  std::string left_end_effector_link_;
  std::string right_end_effector_link_;
  std::string scene_index_file_;
  std::uint32_t default_scene_id_;
  double planning_time_;
  unsigned int planning_attempts_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double position_tolerance_;
  double orientation_tolerance_;
  double gripper_move_duration_;
  double gripper_release_duration_;
  double gripper_effort_;
  double gripper_contact_opening_threshold_;
  double gripper_contact_goal_tolerance_;
  double gripper_contact_goal_time_tolerance_;
  bool treat_gripper_contact_abort_as_success_;
  double grasp_ellipsoid_center_z_;
  bool execute_trajectory_;
  shape_msgs::msg::Mesh grasp_ellipsoid_mesh_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<MoveGroupInterface> left_move_group_;
  std::unique_ptr<MoveGroupInterface> right_move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  std::unique_ptr<StaticSceneLoader> static_scene_loader_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cancel_callback_group_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::Service<PlanToPose>::SharedPtr service_;
  rclcpp::Service<PlanToJoints>::SharedPtr joint_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  FollowJointTrajectoryClient::SharedPtr left_trajectory_client_;
  FollowJointTrajectoryClient::SharedPtr right_trajectory_client_;
  FollowJointTrajectoryClient::SharedPtr left_gripper_client_;
  FollowJointTrajectoryClient::SharedPtr right_gripper_client_;
  rclcpp::Client<GetPlanningScene>::SharedPtr get_planning_scene_client_;
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> request_active_{false};
};

}  // namespace path_planning_server
