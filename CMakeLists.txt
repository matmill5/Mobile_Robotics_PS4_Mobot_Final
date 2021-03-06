cmake_minimum_required(VERSION 2.8.3)
project(mobot_controller)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.10 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(traj_builder src/traj_builder.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(lidar_alarm_mobot src/lidar_alarm_mobot.cpp)
cs_add_executable(current_state_publisher src/current_state_publisher.cpp)
cs_add_executable(modal_trajectory_controller src/modal_trajectory_controller.cpp)
cs_add_executable(des_state_publisher_service src/des_state_publisher_service.cpp)
cs_add_executable(navigation_coordinator src/navigation_coordinator.cpp)
cs_add_executable(lin_steering_wrt_odom src/lin_steering_wrt_odom.cpp)
cs_add_executable(cmd_vel_test src/cmd_vel_test.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(des_state_publisher_service traj_builder)
target_link_libraries(navigation_coordinator traj_builder)

cs_install()
cs_export()
    