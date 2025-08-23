cmake_minimum_required(VERSION 3.5)
project(object_pose_interface_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add message files
set(msg_files
   "msg/Classification.msg"
   "msg/KeypointDetection3D.msg"
   "msg/KeypointDetections3D.msg"
   "msg/ObjectClass.msg"
   "msg/SemanticMapObject.msg"
   "msg/SemanticMapObjectArray.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs 
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
)



ament_package()
