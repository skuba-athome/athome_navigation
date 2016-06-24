cmake_minimum_required(VERSION 2.8.3)
project(athome_move_base)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        rospy
        std_msgs
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        nav_core
        tf
)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

add_service_files(
    FILES
    ClearPointCostmap.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# move_base
add_library(athome_move_base
  src/move_base.cpp
)
target_link_libraries(athome_move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(athome_move_base move_base_gencfg)
add_dependencies(athome_move_base geometry_msgs_gencpp)

add_executable(move_base_node
  src/move_base_node.cpp
)
target_link_libraries(move_base_node athome_move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME athome_move_base)

install(
    TARGETS
        athome_move_base
        move_base_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
