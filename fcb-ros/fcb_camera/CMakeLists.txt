cmake_minimum_required(VERSION 3.16)
project(fcb_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(fcb_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(AV REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

# GStreamer is optional: it is the route to the Jetson hardware encoder
# (nvv4l2h265enc). Without it the libav backend is used.
option(FCB_WITH_GSTREAMER "Build the GStreamer encoder backend" ON)
set(FCB_HAVE_GSTREAMER OFF)
if(FCB_WITH_GSTREAMER)
  pkg_check_modules(GST IMPORTED_TARGET gstreamer-1.0>=1.14 gstreamer-app-1.0)
  if(GST_FOUND)
    set(FCB_HAVE_GSTREAMER ON)
  else()
    message(STATUS "GStreamer dev packages not found; building without the GStreamer backend "
                   "(apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev to enable)")
  endif()
endif()

add_library(${PROJECT_NAME}_component SHARED
  src/visca.cpp
  src/visca_link.cpp
  src/v4l2_capture.cpp
  src/libav_encoder.cpp
  src/gst_encoder.cpp
  src/fcb_camera_node.cpp)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_component PkgConfig::AV)
if(FCB_HAVE_GSTREAMER)
  target_compile_definitions(${PROJECT_NAME}_component PRIVATE FCB_HAVE_GSTREAMER=1)
  target_link_libraries(${PROJECT_NAME}_component PkgConfig::GST)
endif()
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp rclcpp_components sensor_msgs std_msgs std_srvs camera_info_manager
  ffmpeg_image_transport_msgs fcb_interfaces)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "fcb_camera::FcbCameraNode")

add_executable(fcb_camera_node src/fcb_camera_main.cpp)
target_link_libraries(fcb_camera_node ${PROJECT_NAME}_component)
ament_target_dependencies(fcb_camera_node rclcpp)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS fcb_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config udev DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/fcb_bag_to_mp4.py
  scripts/fcb_udev_setup.sh
  scripts/fcb_record.sh
  test/fake_visca_camera.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_visca test/test_visca.cpp)
  target_link_libraries(test_visca ${PROJECT_NAME}_component)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_dependencies(rclcpp sensor_msgs std_srvs camera_info_manager ffmpeg_image_transport_msgs fcb_interfaces)
ament_package()
