#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <algorithm>
#include <string>
#include <thread>
#include <vector>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>


using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto move_group_node = std::make_shared<rclcpp::Node>(
    "move_arm_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = move_group_node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread spin_thread([&executor]() { executor.spin(); });


  MoveGroupInterface arm(move_group_node, "arm");
  MoveGroupInterface gripper(move_group_node, "gripper");

  const std::string tip_link = "Link_5";

  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);
  arm.setGoalPositionTolerance(0.03);



  RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End-effector link: %s", arm.getEndEffectorLink().c_str());

  const auto joint_names = arm.getJointNames();
  const auto joint_values = arm.getCurrentJointValues();
  RCLCPP_INFO(logger, "Current arm joints:");
  for (size_t i = 0; i < std::min(joint_names.size(), joint_values.size()); ++i) {
    RCLCPP_INFO(logger, "  %s = %.3f rad", joint_names[i].c_str(), joint_values[i]);
  }

  const auto current_pose = arm.getCurrentPose(tip_link);
  RCLCPP_INFO(
    logger,
    "Current %s position in %s: x=%.3f y=%.3f z=%.3f",
    tip_link.c_str(),
    current_pose.header.frame_id.c_str(),
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z);


// Create collision object for the robot to avoid
auto const collision_objects = [frame_id =
                                 arm.getPlanningFrame()] {
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects ;
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = frame_id;
  table.id = "table";

  shape_msgs::msg::SolidPrimitive table_box;
  table_box.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_box.dimensions.resize(3);
  table_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 1.0;
  table_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.30;
  table_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.06;

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.0;
  table_pose.position.y = -0.50;
  table_pose.position.z = 0.35;

  table.primitives.push_back(table_box);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  collision_objects.push_back(table);

  // moveit_msgs::msg::CollisionObject red_cube;
  // red_cube.header.frame_id = frame_id;
  // red_cube.id = "red_cube";

  // shape_msgs::msg::SolidPrimitive cube_box;
  // cube_box.type = shape_msgs::msg::SolidPrimitive::BOX;
  // cube_box.dimensions.resize(3);
  // cube_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.05;
  // cube_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.05;
  // cube_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.05;

  // geometry_msgs::msg::Pose cube_pose;
  // cube_pose.orientation.w = 1.0;
  // cube_pose.position.x = -0.23;
  // cube_pose.position.y = -0.55;
  // cube_pose.position.z = 0.395;

  // red_cube.primitives.push_back(cube_box);
  // red_cube.primitive_poses.push_back(cube_pose);
  // red_cube.operation = red_cube.ADD;

  // collision_objects.push_back(red_cube);

  return collision_objects;
}();
  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(collision_objects);

  gripper.setNamedTarget("gripper_open");
  gripper.move();

  arm.setStartStateToCurrentState();
  arm.setPositionTarget(-0.23, -0.55, 0.50, tip_link);


  MoveGroupInterface::Plan plan;
  MoveGroupInterface::Plan gripper_plan;

  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm.execute(plan);
  } else {
    RCLCPP_WARN(logger, "Could not reach target");
  }

  if (gripper.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    gripper.execute(gripper_plan);
  } else {
    RCLCPP_WARN(logger, "Could not reach target");
  }

  gripper.setNamedTarget("gripper_closed");
  // gripper.move();


  if (gripper.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    gripper.execute(gripper_plan);
  } else {
    RCLCPP_WARN(logger, "Could not reach target");
  }


  RCLCPP_INFO(logger, "Demo complete.");
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
