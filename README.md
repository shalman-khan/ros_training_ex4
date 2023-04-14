## ROS-I Training Exercise 4: Service Client to Spawn a Collision Object in RViz


The objective is to spawn a Aruco Tagged Collision Object in RViz. Please refer to the video.



https://user-images.githubusercontent.com/103028647/231994294-d8bb6591-630d-4a1a-bfb6-40fb1ec869f2.mp4


<br>

<br>

### Create a Workspace 

```
mkdir training_ws
cd training_ws && mkdir src && cd src
```

### Clone the repository into src

```
git clone https://github.com/shalman-khan/ros_training_ex4.git
```
<br>

### Environment Setup for Moveit2 [This process may take a while]

```
sudo apt install ros-humble-moveit 
cd ros_training_ex4
chmod +x moveit2_env_setup.sh
./moveit2_env_setup.sh
```

Write a Service client from scratch in moveit_test_node.cpp located in moveit_exercise package [src]
  1) Create a service client for spawn_collision_object service to spawn Aruco Marked Collision Object in RViz
  2) Use transformPose function to determine the pose of Aruco Marked Collision Object with respect to Static Camera
  3) Use spawnCollisionInScene function to spawn collision object in the planning scene.
  4) Use setTargetPoseWithOffset function to set target pose based on Aruco Marked Collision Object with offset
  
### To test the outcome:

Terminal 1:
```
cd ~/training_ws
source install/setup.bash
ros2 launch cam_launch_pkg cam.launch.py
```

Terminal 2:
```
cd ~/training_ws
source install/setup.bash

ros2 run ros2_aruco aruco_node

# For Simulation Mode
ros2 run ros2_aruco aruco_node --ros-args -p sim:=true

```

Terminal 3:
```
cd ~/training_ws
source install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py
```


Terminal 4:
```
cd ~/training_ws
source install/setup.bash
ros2 launch moveit_exercise moveit_collision_client.launch.py
```
