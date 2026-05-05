#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_arm_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End-effector link: %s", arm.getEndEffectorLink().c_str());

  // 1. Go to the named "ready" pose
  RCLCPP_INFO(logger, "Moving to 'ready'...");
  arm.setNamedTarget("ready");
  MoveGroupInterface::Plan plan;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Plan to 'ready' failed");
  }

  // 2. Move the end-effector to a Cartesian pose
  RCLCPP_INFO(logger, "Moving to a target pose...");
  geometry_msgs::msg::Pose target;
  target.position.x = 0.20;
  target.position.y = 0.0;
  target.position.z = 0.40;
  target.orientation.w = 1.0;
  arm.setPoseTarget(target);
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm.execute(plan);
  } else {
    RCLCPP_WARN(logger, "Pose target was not reachable; skipping");
  }

  // 3. Back to home
  RCLCPP_INFO(logger, "Returning to 'home'...");
  arm.setNamedTarget("home");
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm.execute(plan);
  }

  RCLCPP_INFO(logger, "Demo complete.");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
