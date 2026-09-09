#include <rclcpp/rclcpp.hpp>

#include "fcb_camera/fcb_camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<fcb_camera::FcbCameraNode>();
    // Two threads: one for the state timer, one so a slow service call
    // (a zoom with wait=true) does not block the periodic state publisher.
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("fcb_camera"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
