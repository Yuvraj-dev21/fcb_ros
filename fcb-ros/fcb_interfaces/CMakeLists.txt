cmake_minimum_required(VERSION 3.16)
project(fcb_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraState.msg"
  "srv/SetZoom.srv"
  "srv/SetFocus.srv"
  "srv/SetIcr.srv"
  "srv/SetExposure.srv"
  "srv/SetWhiteBalance.srv"
  "srv/SetPicture.srv"
  "srv/Preset.srv"
  "srv/ViscaCommand.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
