#include <memory>

#include "piper_trajectory_bridge/trajectory_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto bridge =
    std::make_shared<piper_trajectory_bridge::PiperTrajectoryBridge>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(bridge);
  executor.spin();
  executor.remove_node(bridge);
  bridge.reset();

  rclcpp::shutdown();
  return 0;
}
