#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncLocalization;
class ExtensionModule;
class GenericTopicSubscription;

/**
 * @brief ROS2 node for localization using pre-built global map
 *
 * This node provides robot localization on a given map, using LiDAR data
 * and optionally IMU data. It runs asynchronously and publishes the
 * estimated pose continuously.
 */
class GlimLocalizationROS : public rclcpp::Node {
public:
  GlimLocalizationROS(const rclcpp::NodeOptions& options);
  ~GlimLocalizationROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t points_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void wait(bool auto_quit = false);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>&
  extension_subscriptions();

private:
  void publish_pose();
  void publish_path();
  void publish_diagnostics();

private:
  std::unique_ptr<glim::TimeKeeper>        time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncLocalization> localization;

  bool   keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  bool   dump_on_unload;

  std::string intensity_field, ring_field;
  std::string global_map_path;
  bool        auto_initial_pose;

  // Localization state
  bool                localization_initialized;
  Eigen::Isometry3d   current_pose;
  double              current_confidence;
  nav_msgs::msg::Path trajectory_path;

  // Extension modules
  std::vector<std::shared_ptr<ExtensionModule>>          extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  rclcpp::TimerBase::SharedPtr                                   timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
                                                    pose_cov_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
