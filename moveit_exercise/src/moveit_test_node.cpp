#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros2_aruco_interfaces/srv/spawn_collision_object.hpp> // New include for service message
#include <iostream>
int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_test_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_test_node");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");


  //////////////////////////////////////////////////////////////////////////////////////////////////////
  // Create Client to Spawn Collision Object
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  // Create a service client to get the aruco box pose from the service server [refer previous exercise]
  auto client = node->create_client<ros2_aruco_interfaces::srv::SpawnCollisionObject>("spawn_collision_object");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(logger, "Service not available, waiting again...");
  }

  // Create a Client request to get arcuo box pose
  auto request = std::make_shared<ros2_aruco_interfaces::srv::SpawnCollisionObject::Request>();

  // Call the service and get the box pose
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    auto box_pose = result.get()->obj_pose;
    auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = 0.28;
      msg.position.y = 0.4;  // <---- This value was changed
      msg.position.z = 0.5;
      return msg;
    }();

    move_group_interface.setPoseTarget(target_pose);
    
  // Create collision object for the robot to avoid based on the retrieved aruco box pose
  auto const collision_object = [frame_id =
                                 move_group_interface.getPlanningFrame(),box_pose] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    // Use the box pose obtained from the server
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  } else {
    RCLCPP_ERROR(logger, "Failed to get the box pose from the server!");
    return 1;
  }
  
// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
moveit::planning_interface::MoveGroupInterface::Plan msg;
auto const ok = static_cast<bool>(move_group_interface.plan(msg));
return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
move_group_interface.execute(plan);
} else {
RCLCPP_ERROR(logger, "Planning failed!");
}

// Shutdown ROS
rclcpp::shutdown();
return 0;
}