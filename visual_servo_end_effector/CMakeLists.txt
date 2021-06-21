cmake_minimum_required(VERSION 2.8.3)
project(visual_servo_end_effector)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_subsystem
  velma_core_cs_task_cs_msgs
  eigen_conversions
  rtt_actionlib
  common_interfaces
  subsystem_common
  controller_common
  )

find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
#orocos_use_package( rtt-typekit )

#catkin_package(
#  INCLUDE_DIRS include
#  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm velma_core_cs_task_cs_interface_msgs
#)

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
### Orocos Targets ###

ros_generate_rtt_master()

orocos_component(visual_servo_end_effector
    src/visual_servo_end_effector_subsystem.cpp
    src/predicates.cpp
    src/components/visual_servo_end_effector.cpp
    src/components/ext_commands.cpp
    src/components/manipulation_strategy.cpp
    src/components/visual_servo_fake_end_effector.cpp
    src/components/delay_test.cpp
)
target_link_libraries(visual_servo_end_effector ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} rtt-${PROJECT_NAME}-master)

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm rtt_actionlib
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

#catkin_install_python(PROGRAMS
#    scripts/...
#DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
