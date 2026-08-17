#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "path_planning_server/path_planning_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const auto node =
    std::make_shared<path_planning_server::PathPlanningNode>(options);

  try {
    node->initialize();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Initialization failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
