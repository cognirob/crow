cmake_minimum_required(VERSION 3.5)
project(crow_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Contour.msg"
  "msg/BBox.msg"
  "msg/DetectionBBox.msg"
  "msg/DetectionMask.msg"
  "msg/ActionDetection.msg"
  "msg/DetectionObject.msg"
  "msg/SegmentedPointcloud.msg"
  "msg/ObjectPointcloud.msg"
  #"msg/Particles.msg"
  "msg/PclDimensions.msg"
  "msg/FilteredPose.msg"
  "msg/ProcessedSentence.msg"
  "msg/StringList.msg"
  "msg/SentenceProgram.msg"
  "msg/StampedString.msg"
  "msg/CommandType.msg"
  "msg/MarkerMsg.msg"
  #"msg/StopStatus.msg"
  #"msg/GripperStatus.msg"
  #"msg/RobotStatus.msg"
  #"msg/RobotPose.msg"
  "msg/ObjectType.msg"
  "msg/NlpStatus.msg"
  "msg/Runtime.msg"
  "msg/AssemblyObjectProbability.msg"
  "msg/AssemblyActionProbability.msg"

  #"srv/GetRobotStatus.srv"

  #"action/PickNPlace.action"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
