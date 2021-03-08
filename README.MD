# Mobile Robotics - PS4 - Mobot Path Execution

# Content - ROS Nodes
current_state_publisher
des_state_publisher_service
lidar_alarm_mobot
modal_trajectory_controller
navigation_coordinator
traj_builder

# Usage
roslaunch mobot_urdf mobot_in_pen.launch
rosrun mobot_controller current_state_publisher
rosrun mobot_controller lidar_alarm_mobot
rosrun mobot_controller modal_trajectory_controller
rosrun mobot_controller des_state_publisher_service
rosrun mobot_controller navigation_coordinator

#### Trouble
- For `lidar_alarm_mobot` node, there was a bug in the mobot lidar sensor that can not perform the math correctly (without the additional loop to reassign the value of limit from 0 to 89, the value changing weirdly depending on the lidar distance even though no related variable was used in the calculation). We solve it by assing the other half and copy it to the other side.