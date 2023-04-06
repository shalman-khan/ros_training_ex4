#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros2_aruco_interfaces/srv/spawn_collision_object.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


/**
* Transforms a pose with respect to a reference pose.
* 
* @param pose - the pose to be transformed. Must be in geometry_msgs:pose format.
* @param referencePose - the pose to be transformed to. ust be in geometry_msgs:pose format.
*/
geometry_msgs::msg::Pose transformPose(geometry_msgs::msg::Pose pose, geometry_msgs::msg::Pose referencePose)
{
  tf2::Transform tfPose, tfReferencePose;
  tf2::fromMsg(pose, tfPose);
  tf2::fromMsg(referencePose, tfReferencePose);

  tf2::Transform tfTransform = tfReferencePose.inverse() * tfPose;

  geometry_msgs::msg::Pose transformedPose;
  tf2::toMsg(tfTransform, transformedPose);

  return transformedPose;
}


/**
* Spawns a collision object in the planning scene. This is used to determine whether or not an aruco box is in front of the ground or not
* 
* @param planning_scene_interface - The planning scene interface to spawn the collision object in the scene
* @param frame_id - The frame id of the aruco box we're spawning
* @param box_pose_wrt_camera - The pose of the aruco collision box we're spawning with respect to camera.
*/
void spawnCollisionInScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const std::string& frame_id, const geometry_msgs::msg::Pose& box_pose_wrt_camera)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "collision_aruco_box";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 0.055;
  primitive.dimensions[primitive.BOX_Z] = 0.040;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose_wrt_camera);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);
}

/**
* Set the PoseTarget with an offset
* 
* @param box_pose_wrt_camera - the pose we want to target
* @param move_group_interface - the move group interface to which the target is set
*/
void setTargetPoseWithOffset(const geometry_msgs::msg::Pose& box_pose_wrt_camera,
                             moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    // Target Pose set with offset to determined aruco box pose
    geometry_msgs::msg::Pose target_pose = box_pose_wrt_camera;
    target_pose.position.x = target_pose.position.x - 0.3;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.707;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.707;

    // Set the pose target of the move group.
    move_group_interface.setPoseTarget(target_pose);
}


int main(int argc, char * argv[])
{
  // \ brief Initializes rclcpp client library and sets command line options to parse arguments
  rclcpp::init(argc, argv);

  // \ brief Declaration of Moveit test node with node options
  auto node = std::make_shared<rclcpp::Node>(
    "moveit_test_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // \ brief ROS logger creation
  auto logger = rclcpp::get_logger("moveit_test_node");


  // \ brief MoveIt MoveGroup Interface Creation
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Declaration of Static Camera Pose in the scene
  geometry_msgs::msg::Pose static_camera_pose;
  static_camera_pose.position.x = 1.0;
  static_camera_pose.position.y = 0.0;
  static_camera_pose.position.z = 1.0;
  static_camera_pose.orientation.x = 0.0;
  static_camera_pose.orientation.y = 1.0;
  static_camera_pose.orientation.z = 0.0;
  static_camera_pose.orientation.w = 0.0;

  /**
  Write a Service Client to Spawn the Aruco Marked Collision Object in RViz

  1) Create a service client for spawn_collision_object service to spawn Aruco Marked Collision Object in RViz
  2) Use transformPose function to determine the pose of Aruco Marked Collision Object with respect to Static Camera
  3) Use spawnCollisionInScene function to spawn collision object in the planning scene.
  4) Use setTargetPoseWithOffset function to set target pose based on Aruco Marked Collision Object with offset
  **/

  auto client = node->create_client<ros2_aruco_interfaces::srv::SpawnCollisionObject>("spawn_collision_object");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(logger, "Service not available, waiting again...");
  }

  auto request = std::make_shared<ros2_aruco_interfaces::srv::SpawnCollisionObject::Request>();

  auto result = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {

    auto box_pose = result.get()->obj_pose;
    auto box_pose_wrt_camera = transformPose(box_pose, static_camera_pose);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    spawnCollisionInScene(planning_scene_interface, move_group_interface.getPlanningFrame(), box_pose_wrt_camera);
    setTargetPoseWithOffset(box_pose_wrt_camera, move_group_interface);
    
  } else {
    RCLCPP_ERROR(logger, "Failed to get the collision object pose from the server!");
    return 1;
  }

  // Plan Execution
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}