cmake_minimum_required(VERSION 2.8.3)
project(upo_nav_irl)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

#set (CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
	angles
	cmake_modules
	geometry_msgs
	nav_msgs
	roscpp
	pluginlib
	actionlib
	rosconsole
	message_generation
	nav_core
	tf
	std_msgs
	visualization_msgs
	upo_msgs
	rosbag
	upo_rrt_planners
	move_base_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED
    COMPONENTS
        thread
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})


# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/upo_navigation.cfg
#)


## Generate messages in the 'msg' folder
#add_message_files(
#    DIRECTORY msg
#    FILES
#    Position2DInt.msg
#	PointCost.msg
#	FeaturesWeights.msg
# )


#add_service_files(
#   FILES
#   PointCosts.srv
#)

#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
	angles
	cmake_modules
	geometry_msgs
	nav_msgs
	roscpp
	pluginlib
	actionlib
	rosconsole
	roscpp
	#message_generation
	message_runtime 
	nav_core
	tf
	rospy
	std_msgs
	visualization_msgs
	upo_msgs
    navigation_features
	upo_rrt_planners
	rosbag
	#DEPENDS system_lib
)




#---learning  -----------------------------------
add_executable(upo_nav_irl
  src/Learning.cpp
  src/Learner.cpp
  src/Algorithms/RTIRL.cpp
  src/Algorithms/RLT.cpp
  src/ros/PlanningRos.cpp
  src/ros/DataManagementRos.cpp
  src/ros/RunLearningRos.cpp
)

add_dependencies(upo_nav_irl upo_msgs_gencpp)
add_dependencies(upo_nav_irl upo_rrt_planners_gencpp)

target_link_libraries(upo_nav_irl
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${Eigen_LIBRARIES}
	upo_navigation
	upo_rrt_planners_ros
    )
set_target_properties(upo_nav_irl PROPERTIES OUTPUT_NAME upo_nav_irl)

add_dependencies(upo_nav_irl ${catkin_EXPORTED_TARGETS})


#---Evaluation  -----------------------------------
add_executable(evaluate_paths
  src/ros/EvaluatePaths.cpp
  src/ros/PlanningRos.cpp
  src/ros/DataManagementRos.cpp
)

add_dependencies(evaluate_paths upo_msgs_gencpp)
add_dependencies(evaluate_paths upo_rrt_planners_gencpp)

target_link_libraries(evaluate_paths
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${Eigen_LIBRARIES}
	upo_rrt_planners_ros
	upo_rrt_planners3_ros
    )
set_target_properties(evaluate_paths PROPERTIES OUTPUT_NAME evaluate_paths)

add_dependencies(evaluate_paths ${catkin_EXPORTED_TARGETS})






