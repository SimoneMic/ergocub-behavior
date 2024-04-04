################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the custom_leaf_nodes library
add_library(custom_leaf_nodes STATIC)

option(USE_ROS "Use the ROS2 instead of YARP when communicating with the navigation stack" OFF)
if(USE_ROS)
    add_definitions(-DUSE_ROS)
endif()

add_subdirectory(interfaces)
add_subdirectory(mock_components)

if(USE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(action_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
endif(USE_ROS)

find_package(eCubPerceptionInterface REQUIRED)
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        actions
        conditions
)

file(GLOB BT_NODES "actions/*.cpp" "conditions/*.cpp")
file(GLOB COMPONENTS "interfaces/*/src/*.cpp")

target_sources(custom_leaf_nodes
  PRIVATE
  ${BT_NODES}
  ${COMPONENTS}
  common.cpp
)

if(USE_ROS)
  ament_target_dependencies(custom_leaf_nodes 
    "eCubPerceptionInterface"
    "behaviortree_cpp_v3"
    "YARP"
    "rclcpp"
    "action_msgs"
  )
  #target_link_libraries(custom_leaf_nodes
  #PRIVATE
  #  eCubPerceptionInterface::eCubPerceptionInterface
  #  BT::behaviortree_cpp_v3
  #  YARP::YARP_os
  #  rclcpp::rclcpp
  #  action_msgs::action_msgs
  #  )
else(
  target_link_libraries(custom_leaf_nodes
  PRIVATE
    eCubPerceptionInterface::eCubPerceptionInterface
    BT::behaviortree_cpp_v3
    YARP::YARP_os
    )
    )
endif(USE_ROS)


add_executable(run_bt)
target_sources(run_bt
    PRIVATE
    run_bt.cpp
)

target_link_libraries(run_bt
    PRIVATE
    custom_leaf_nodes
    YARP::YARP_os
    YARP::YARP_init
    YARP::YARP_dev
    )
