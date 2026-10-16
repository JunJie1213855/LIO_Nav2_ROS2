#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <plan_manage/scan_replan_fsm.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node");

  try
  {
    scan_planner::SCANReplanFSM planner;
    planner.init(node.get());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
