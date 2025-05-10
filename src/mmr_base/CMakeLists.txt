cmake_minimum_required(VERSION 3.5)
project(mmr_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sync.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/PerceptionCone.msg"
  "msg/PerceptionCones.msg"
  "msg/ClusterArray.msg"
  "msg/AutsystemState.msg"
  "msg/RaceStatus.msg"
  "msg/TargetSpeed.msg"
  "msg/SteeringActuatorStatus.msg"
  "msg/BrakingActuatorStatus.msg"
  "msg/ActuatorsStatus.msg"
  "msg/VehicleState.msg"
  "msg/NavSatHeading.msg"
  "msg/GpsSpeed.msg"
  "msg/CarTargetInfo.msg"
  "msg/SpeedProfilePoint.msg"
  "msg/SpeedProfilePoints.msg"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  "msg/UVCoordinate.msg"
  "msg/MeshFile.msg"
  "msg/CmdMotor.msg"
  "msg/ActuatorStatus.msg"
  "msg/EcuStatus.msg"
  "msg/ResStatus.msg"
  "msg/CmdEcu.msg"
  "msg/ControlLog.msg"
  "msg/PurePursuitLog.msg"
  # Add custom msg dependecies to standard ROS2 msgs
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs ackermann_msgs
 )

# Install launch directory
install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/car_configurations.h
  config/configuration.hpp
  DESTINATION include/${PROJECT_NAME}
)
# Install config directory
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips tCMAKE_PREFIX_PATHhe linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
