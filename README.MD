# mobot_controller

## Package name: mobot_controller

### Note: 

The math convention was fixed so that the `traj_builder.cpp` and the `mobot_urdf` + `gazebo`.
### Setup and Command:

- Run `Gazebo` and spawn `mobot`:

`roslaunch mobot_urdf mobot_in_pen.lauch`

- Run nodes following this order:

`rosrun mobot_controller current_state_publisher`

`rosrun mobot_controller lidar_alarm_mobot`

`rosrun mobot_controller modal_trajectory_controller`

`rosrun mobot_controller des_state_publisher_service`

- Run the main node:

`rosrun mobot_controller navigation_coordinator`