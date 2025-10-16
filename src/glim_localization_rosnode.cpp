#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_localization_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions                       options;

  auto glim_loc = std::make_shared<glim::GlimLocalizationROS>(options);

  rclcpp::spin(glim_loc);
  rclcpp::shutdown();

  std::string dump_path = "/tmp/localization_dump";
  glim_loc->declare_parameter<std::string>("dump_path", dump_path);
  glim_loc->get_parameter<std::string>("dump_path", dump_path);

  glim_loc->wait();
  glim_loc->save(dump_path);

  return 0;
}
