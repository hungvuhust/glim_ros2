#include <glim_ros/glim_localization_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <thread>
#include <iostream>
#include <functional>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <glim/util/debug.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/localization/async_localization.hpp>
#include <glim/localization/localization_base.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/ros_qos.hpp>

namespace glim {

GlimLocalizationROS::GlimLocalizationROS(const rclcpp::NodeOptions& options)
  : Node("glim_localization_ros", options),
    localization_initialized(false),
    current_pose(Eigen::Isometry3d::Identity()),
    current_confidence(0.0) {
  // Setup logger
  auto logger = spdlog::stdout_color_mt("glim");
  logger->sinks().push_back(get_ringbuffer_sink());
  spdlog::set_default_logger(logger);

  bool debug = false;
  this->declare_parameter<bool>("debug", false);
  this->get_parameter<bool>("debug", debug);

  if (debug) {
    spdlog::info("enable debug printing");
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      "/tmp/glim_localization_log.log", true);
    logger->sinks().push_back(file_sink);
    logger->set_level(spdlog::level::trace);

    print_system_info(logger);
  }

  dump_on_unload = false;
  this->declare_parameter<bool>("dump_on_unload", false);
  this->get_parameter<bool>("dump_on_unload", dump_on_unload);

  if (dump_on_unload) {
    spdlog::info("dump_on_unload={}", dump_on_unload);
  }

  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path =
      ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  logger->info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_loc_ros(
    glim::GlobalConfig::get_config_path("config_localization_ros"));

  keep_raw_points    = config_loc_ros.param<bool>("glim_localization_ros",
                                               "keep_raw_points",
                                               false);
  imu_time_offset    = config_loc_ros.param<double>("glim_localization_ros",
                                                 "imu_time_offset",
                                                 0.0);
  points_time_offset = config_loc_ros.param<double>("glim_localization_ros",
                                                    "points_time_offset",
                                                    0.0);
  acc_scale =
    config_loc_ros.param<double>("glim_localization_ros", "acc_scale", 1.0);

  glim::Config config_sensors(
    glim::GlobalConfig::get_config_path("config_sensors"));
  intensity_field = config_sensors.param<std::string>("sensors",
                                                      "intensity_field",
                                                      "intensity");
  ring_field = config_sensors.param<std::string>("sensors", "ring_field", "");

  // Get global map path
  this->declare_parameter<std::string>("global_map_path", "");
  this->get_parameter<std::string>("global_map_path", global_map_path);

  if (global_map_path.empty()) {
    spdlog::critical("global_map_path is not set!");
    throw std::runtime_error("global_map_path parameter is required");
  }

  // Auto initial pose (always use identity for immediate start)
  this->declare_parameter<bool>("auto_initial_pose", true);
  this->get_parameter<bool>("auto_initial_pose", auto_initial_pose);

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Load localization module
  glim::Config config_localization(
    glim::GlobalConfig::get_config_path("config_localization"));
  const std::string localization_so_name =
    config_localization.param<std::string>("localization",
                                           "so_name",
                                           "liblocalization_cpu.so");
  spdlog::info("load {}", localization_so_name);

  std::shared_ptr<glim::LocalizationBase> loc =
    glim::LocalizationBase::load_module(localization_so_name);
  if (!loc) {
    spdlog::critical("failed to load localization module");
    throw std::runtime_error("Failed to load localization module");
  }

  // Create async localization
  localization.reset(new glim::AsyncLocalization(loc));

  // Load global map
  spdlog::info("Loading global map from: {}", global_map_path);
  if (!localization->load_global_map(global_map_path)) {
    spdlog::critical("failed to load global map from {}", global_map_path);
    throw std::runtime_error("Failed to load global map");
  }
  spdlog::info("Global map loaded successfully");

  // Always set initial pose for immediate start
  if (auto_initial_pose) {
    spdlog::info("Using identity as initial pose (auto mode enabled)");
    if (!localization->set_initial_pose(Eigen::Isometry3d::Identity())) {
      spdlog::critical("failed to set initial pose");
      throw std::runtime_error("Failed to set initial pose");
    }
    localization_initialized = true;
  } else {
    spdlog::info(
      "Manual initial pose mode - localization will start after receiving "
      "initial pose");
  }

  // Extension modules
  const auto extensions =
    config_loc_ros.param<std::vector<std::string>>("glim_localization_ros",
                                                   "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      spdlog::info("load extension {}", extension);
      auto ext_module = ExtensionModule::load_module(extension);
      if (ext_module == nullptr) {
        spdlog::error("failed to load {}", extension);
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros =
          std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions(*this);
          extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
        }
      }
    }
  }

  // ROS-related
  using std::placeholders::_1;
  const std::string imu_topic =
    config_loc_ros.param<std::string>("glim_localization_ros",
                                      "imu_topic",
                                      "/imu");
  const std::string points_topic =
    config_loc_ros.param<std::string>("glim_localization_ros",
                                      "points_topic",
                                      "/points");
  const std::string image_topic =
    config_loc_ros.param<std::string>("glim_localization_ros",
                                      "image_topic",
                                      "");
  const std::string initial_pose_topic =
    config_loc_ros.param<std::string>("glim_localization_ros",
                                      "initial_pose_topic",
                                      "/initialpose");

  // Subscribers
  rclcpp::SensorDataQoS default_imu_qos;
  default_imu_qos.get_rmw_qos_profile().depth = 1000;
  auto qos                                    = get_qos_settings(config_loc_ros,
                              "glim_localization_ros",
                              "imu_qos",
                              default_imu_qos);
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, qos, std::bind(&GlimLocalizationROS::imu_callback, this, _1));

  qos = get_qos_settings(config_loc_ros, "glim_localization_ros", "points_qos");
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    points_topic,
    qos,
    std::bind(&GlimLocalizationROS::points_callback, this, _1));

  // Initial pose subscriber
  initial_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic,
      10,
      std::bind(&GlimLocalizationROS::initial_pose_callback, this, _1));

#ifdef BUILD_WITH_CV_BRIDGE
  if (!image_topic.empty()) {
    qos =
      get_qos_settings(config_loc_ros, "glim_localization_ros", "image_qos");
    image_sub = image_transport::create_subscription(
      this,
      image_topic,
      std::bind(&GlimLocalizationROS::image_callback, this, _1),
      "raw",
      qos.get_rmw_qos_profile());
  }
#endif

  for (const auto& sub : this->extension_subscriptions()) {
    spdlog::debug("subscribe to {}", sub->topic);
    sub->create_subscriber(*this);
  }

  // Publishers
  pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10);
  pose_cov_pub =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_cov", 10);
  path_pub =
    this->create_publisher<nav_msgs::msg::Path>("/localization/path", 10);

  // Initialize path message
  trajectory_path.header.frame_id = "map";

  // Start timer
  timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                  [this]() { timer_callback(); });

  spdlog::info("GlimLocalizationROS initialized");
}

GlimLocalizationROS::~GlimLocalizationROS() {
  spdlog::debug("quit");
  extension_modules.clear();

  if (dump_on_unload) {
    std::string dump_path = "/tmp/localization_dump";
    wait(true);
    save(dump_path);
  }
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>&
GlimLocalizationROS::extension_subscriptions() {
  return extension_subs;
}

void GlimLocalizationROS::initial_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  spdlog::info("Received initial pose: [{:.2f}, {:.2f}, {:.2f}]",
               msg->pose.pose.position.x,
               msg->pose.pose.position.y,
               msg->pose.pose.position.z);

  // Convert ROS pose to Eigen
  Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
  initial_pose.translation()     = Eigen::Vector3d(msg->pose.pose.position.x,
                                               msg->pose.pose.position.y,
                                               msg->pose.pose.position.z);

  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                       msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z);
  initial_pose.linear() = q.toRotationMatrix();

  // Set initial pose in localization
  if (localization->set_initial_pose(initial_pose)) {
    localization_initialized = true;
    current_pose             = initial_pose;
    spdlog::info("Initial pose set successfully");

    // Clear trajectory path
    trajectory_path.poses.clear();
  } else {
    spdlog::error("Failed to set initial pose");
  }
}

void GlimLocalizationROS::imu_callback(
  const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  // Note: AsyncLocalization currently doesn't use IMU data directly
  // This is a placeholder for future IMU integration if needed
}

#ifdef BUILD_WITH_CV_BRIDGE
void GlimLocalizationROS::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  spdlog::trace("image: {}.{}",
                msg->header.stamp.sec,
                msg->header.stamp.nanosec);

  // Note: AsyncLocalization currently doesn't use image data directly
  // This is a placeholder for future image integration if needed
}
#endif

size_t GlimLocalizationROS::points_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  spdlog::trace("points: {}.{}",
                msg->header.stamp.sec,
                msg->header.stamp.nanosec);

  // Skip initialization check - process points immediately
  auto raw_points = glim::extract_raw_points(*msg, intensity_field, ring_field);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }

  raw_points->stamp += points_time_offset;
  time_keeper->process(raw_points);
  auto preprocessed = preprocessor->preprocess(raw_points);

  if (keep_raw_points) {
    preprocessed->raw_points = raw_points;
  }

  localization->insert_frame(preprocessed);

  const size_t workload = localization->workload();
  spdlog::debug("localization workload={}", workload);

  return workload;
}

bool GlimLocalizationROS::needs_wait() {
  for (const auto& ext_module : extension_modules) {
    if (ext_module->needs_wait()) {
      return true;
    }
  }

  return false;
}

void GlimLocalizationROS::timer_callback() {
  for (const auto& ext_module : extension_modules) {
    if (!ext_module->ok()) {
      rclcpp::shutdown();
    }
  }

  if (!localization_initialized) {
    return;
  }

  // Get localization results
  std::vector<glim::EstimationFrame::ConstPtr> localization_results;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  localization->get_results(localization_results, marginalized_frames);

  // Update current state
  if (localization->is_initialized()) {
    current_pose       = localization->get_current_pose();
    current_confidence = localization->get_localization_confidence();

    // Publish pose
    publish_pose();

    // Publish path
    if (!localization_results.empty()) {
      publish_path();
    }

    // Publish diagnostics
    publish_diagnostics();
  }
}

void GlimLocalizationROS::publish_pose() {
  auto now = this->get_clock()->now();

  // Publish PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp    = now;
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = current_pose.translation().x();
  pose_msg.pose.position.y = current_pose.translation().y();
  pose_msg.pose.position.z = current_pose.translation().z();

  Eigen::Quaterniond q(current_pose.rotation());
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();

  pose_pub->publish(pose_msg);

  // Publish PoseWithCovarianceStamped
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
  pose_cov_msg.header    = pose_msg.header;
  pose_cov_msg.pose.pose = pose_msg.pose;

  // Simple covariance based on confidence
  double cov_scale =
    (1.0 - current_confidence) * 0.5;  // Max 0.5m when confidence=0
  for (int i = 0; i < 6; ++i) {
    pose_cov_msg.pose.covariance[i * 7] = cov_scale * cov_scale;
  }

  pose_cov_pub->publish(pose_cov_msg);
}

void GlimLocalizationROS::publish_path() {
  // Add current pose to trajectory
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp    = this->get_clock()->now();
  pose_stamped.header.frame_id = "map";

  pose_stamped.pose.position.x = current_pose.translation().x();
  pose_stamped.pose.position.y = current_pose.translation().y();
  pose_stamped.pose.position.z = current_pose.translation().z();

  Eigen::Quaterniond q(current_pose.rotation());
  pose_stamped.pose.orientation.w = q.w();
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();

  trajectory_path.poses.push_back(pose_stamped);

  // Limit trajectory size
  if (trajectory_path.poses.size() > 10000) {
    trajectory_path.poses.erase(trajectory_path.poses.begin());
  }

  trajectory_path.header.stamp = this->get_clock()->now();
  path_pub->publish(trajectory_path);
}

void GlimLocalizationROS::publish_diagnostics() {
  // Future: Publish detailed diagnostics
  // - Localization confidence
  // - Number of correspondences
  // - Processing time
  // - etc.
}

void GlimLocalizationROS::wait(bool auto_quit) {
  spdlog::info("waiting for localization");
  localization->join();

  if (!auto_quit) {
    bool terminate = false;
    while (!terminate && rclcpp::ok()) {
      for (const auto& ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

void GlimLocalizationROS::save(const std::string& path) {
  spdlog::info("saving localization data to {}", path);
  // Future: Save trajectory, statistics, etc.
  for (auto& module : extension_modules) {
    module->at_exit(path);
  }
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimLocalizationROS);
