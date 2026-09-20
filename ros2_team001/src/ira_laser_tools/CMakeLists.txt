cmake_minimum_required(VERSION 3.5)
project(ira_laser_tools)

# Default to C99
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros
  pcl_ros
  Eigen3
  PCL
)

include_directories(include)


# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
ament_target_dependencies(laserscan_multi_merger ${dependencies})
target_include_directories(laserscan_multi_merger PUBLIC ${EIGEN_INCLUDE_DIRS})
target_include_directories(laserscan_multi_merger PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(laserscan_multi_merger ${PCL_LIBRARIES})

add_executable(laserscan_virtualizer src/laserscan_virtualizer.cpp)
ament_target_dependencies(laserscan_virtualizer ${dependencies})
target_include_directories(laserscan_virtualizer PUBLIC ${EIGEN_INCLUDE_DIRS})
target_include_directories(laserscan_virtualizer PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(laserscan_virtualizer ${PCL_LIBRARIES})

add_library(${PROJECT_NAME} SHARED src/laserscan_multi_merger.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::LaserscanMerger")

install(TARGETS ${PROJECT_NAME} laserscan_multi_merger laserscan_virtualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})


install(TARGETS ${PROJECT_NAME}
EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
      DESTINATION share/${PROJECT_NAME}/cmake)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)
ament_export_libraries(${PROJECT_NAME})

install(
    DIRECTORY include/
    DESTINATION include
)
ament_package()
