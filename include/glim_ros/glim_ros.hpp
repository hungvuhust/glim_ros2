#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <any>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions &options);
  ~GlimROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t
  points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void wait(bool auto_quit = false);
  void save(const std::string &path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>> &
  extension_subscriptions();

  // Localization mode methods
  void relocalization_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response>      response);

  void initial_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ViewerCallbacks handlers
  void on_viewer_user_event(int pose_id);
  void on_viewer_load_map();
  void on_viewer_request_relocalize(const Eigen::Vector3d& pos);

private:
  std::unique_ptr<glim::TimeKeeper>        time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping>         sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping>      global_mapping;

  // Localization mode
  bool              localization_mode;
  Eigen::Isometry3d initial_pose_guess;
  bool              pending_relocalization;  // Flag to trigger relocalization when frames arrive
  bool              prebuilt_map_loaded;     // Flag indicating prebuilt map was loaded
  int               viewer_ready_counter;    // Counter to wait for viewer initialization
  std::string       pending_map_path;        // Map path to load after viewer is ready

  // ViewerCallbacks IDs for cleanup
  int viewer_user_event_id;
  int viewer_load_map_id;
  int viewer_relocalize_id;

  bool   keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  bool   dump_on_unload;

  std::string intensity_field, ring_field;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>>          extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  rclcpp::TimerBase::SharedPtr                                   timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
                                                     initial_pose_sub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalization_service;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

} // namespace glim
