"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import qos_profile_sensor_data
from ros2_aruco import transformations
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.srv import SpawnCollisionObject
from cv_bridge import CvBridge
import numpy as np
import cv2


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        # Declare and read parameters [Aruco Marker Parameters & Camera Parameters]
        self.declare_parameter('marker_size', .055)
        self.declare_parameter('aruco_dictionary_id', 'DICT_5X5_250')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_frame', None)

        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        dictionary_id_name = self.get_parameter('aruco_dictionary_id').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        # Dictionary ID Validation 
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_250):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(f'bad aruco_dictionary_id: {dictionary_id_name}')
            options = '\n'.join([s for s in dir(cv2.aruco) if s.startswith('DICT')])
            self.get_logger().error(f'valid options: {options}')

        # Subscriptions - Camera Info & Marker Detection, Pose Estimation
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        
        # Publishers - Estimated Pose Array
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)

        # Camera Parameters Initialization
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Arcuo Parameters Initializatoin
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.pose = Pose()


# Excercise 3A: Create Service Server to Estiamte Pose Array.
        # Service Server - Estimated Pose Array
        self.srv = self.create_service(SpawnCollisionObject, 'spawn_collision_object', self.spawn_collision_obj_callback)
        
        #Corresponding Service Client Call 
        #ros2 service call /spawn_collision_object ros2_aruco_interfaces/srv/SpawnCollisionObject '{spawn_obj: true}'

# Excercise 3B: Create a Callback to retrieve pose information
    def spawn_collision_obj_callback(self, request, response):
        response.obj_pose = self.pose
        return response


    # Camera Info - Callback
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

    # Marker Detection, Pose Estimation - Callback
    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn('No camera info has been received!')
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')

        pose_array = PoseArray()
        pose_array.header.frame_id = self.camera_frame or self.info_msg.header.frame_id
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)


        if marker_ids is not None:
            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            for i, marker_id in enumerate(marker_ids):
                self.pose.position.x = tvecs[i][0][0]
                self.pose.position.y = tvecs[i][0][1]
                self.pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]

                quat = transformations.quaternion_from_matrix(rot_matrix)

                self.pose.orientation.x = quat[0]
                self.pose.orientation.y = quat[1]
                self.pose.orientation.z = quat[2]
                self.pose.orientation.w = quat[3]

                pose_array.poses.append(self.pose)

        self.poses_pub.publish(pose_array)

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()