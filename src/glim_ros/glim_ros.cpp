#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <boost/format.hpp>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <thread>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

#include <glim/mapping/async_global_mapping.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/localization.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>
#include <glim/viewer/viewer_callbacks.hpp>
// #include <glim/util/debug.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/ros_qos.hpp>

namespace glim {

GlimROS::GlimROS(const rclcpp::NodeOptions &options)
    : Node("glim_ros", options), localization_mode(false),
      initial_pose_guess(Eigen::Isometry3d::Identity()),
      pending_relocalization(false), prebuilt_map_loaded(false),
      viewer_ready_counter(0), viewer_user_event_id(-1), viewer_load_map_id(-1),
      viewer_relocalize_id(-1) {
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
        "/tmp/glim_log.log", true);
    logger->sinks().push_back(file_sink);
    logger->set_level(spdlog::level::trace);

    // print_system_info(logger);
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
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" +
                  config_path;
  }

  logger->info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  keep_raw_points =
      config_ros.param<bool>("glim_ros", "keep_raw_points", false);
  imu_time_offset =
      config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  points_time_offset =
      config_ros.param<double>("glim_ros", "points_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 1.0);

  glim::Config config_sensors(
      glim::GlobalConfig::get_config_path("config_sensors"));
  intensity_field = config_sensors.param<std::string>(
      "sensors", "intensity_field", "intensity");
  ring_field = config_sensors.param<std::string>("sensors", "ring_field", "");

  // Setup GPU-based linearization
#ifdef BUILD_GTSAM_POINTS_GPU
  gtsam_points::LinearizationHook::register_hook(
      []() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_odometry(
      glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string odometry_estimation_so_name =
      config_odometry.param<std::string>("odometry_estimation", "so_name",
                                         "libodometry_estimation_cpu.so");
  spdlog::info("load {}", odometry_estimation_so_name);

  std::shared_ptr<glim::OdometryEstimationBase> odom =
      OdometryEstimationBase::load_module(odometry_estimation_so_name);
  if (!odom) {
    spdlog::critical("failed to load odometry estimation module");
    abort();
  }
  odometry_estimation.reset(
      new glim::AsyncOdometryEstimation(odom, odom->requires_imu()));

  // Sub mapping
  if (config_ros.param<bool>("glim_ros", "enable_local_mapping", true)) {
    const std::string sub_mapping_so_name =
        glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping"))
            .param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
    if (!sub_mapping_so_name.empty()) {
      spdlog::info("load {}", sub_mapping_so_name);
      auto sub = SubMappingBase::load_module(sub_mapping_so_name);
      if (sub) {
        sub_mapping.reset(new AsyncSubMapping(sub));
      }
    }
  }

  // Store localization mode flag
  localization_mode =
      config_ros.param<bool>("glim_ros", "localization_mode", false);

  // Extention modules - Load BEFORE global mapping to ensure callbacks are
  // registered This is critical for localization mode where pre-built map needs
  // to notify viewers In localization mode, read from "localization" section to
  // get localization-specific extensions
  const std::string section = localization_mode ? "localization" : "glim_ros";
  const auto        extensions =
      config_ros.param<std::vector<std::string>>(section, "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto &extension : *extensions) {
      if (extension.find("viewer") == std::string::npos &&
          extension.find("monitor") == std::string::npos) {
        spdlog::warn("Extension modules are enabled!!");
        spdlog::warn(
            "You must carefully check and follow the licenses of ext modules");

        try {
          const std::string config_ext_path =
              ament_index_cpp::get_package_share_directory("glim_ext") +
              "/config";
          spdlog::info("config_ext_path: {}", config_ext_path);
          glim::GlobalConfig::instance()->override_param<std::string>(
              "global", "config_ext", config_ext_path);
        } catch (ament_index_cpp::PackageNotFoundError &e) {
          spdlog::warn("glim_ext package path was not found!!");
        }

        break;
      }
    }

    for (const auto &extension : *extensions) {
      spdlog::info("load {}", extension);
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

  // Global mapping or Localization - Initialize module
  // In localization mode, map loading is deferred to timer_callback
  if (config_ros.param<bool>("glim_ros", "enable_global_mapping", true)) {
    std::string                        so_name;
    std::shared_ptr<GlobalMappingBase> global;

    if (localization_mode) {
      // Localization mode - defer map loading to timer_callback
      spdlog::info("Starting in localization mode");
      so_name =
          glim::Config(
              glim::GlobalConfig::get_config_path("config_global_mapping"))
              .param<std::string>("global_mapping", "localization_so_name",
                                  "liblocalization.so");

      // Store map path for deferred loading
      pending_map_path =
          config_ros.param<std::string>("glim_ros", "map_path", "");
      if (pending_map_path.empty()) {
        spdlog::critical(
            "localization_mode is enabled but map_path is not set!");
        abort();
      }

      if (pending_map_path[0] != '/') {
        // map_path is relative to the glim directory
        pending_map_path = ament_index_cpp::get_package_share_directory("glim") + "/" +
                   pending_map_path;
      }

      spdlog::info("Pre-built map will be loaded from: {}", pending_map_path);
      spdlog::info("load {}", so_name);

      global = GlobalMappingBase::load_module(so_name);
      if (global) {
        // Don't load map here - will load in timer_callback after viewer ready
        global_mapping.reset(new AsyncGlobalMapping(global));
        spdlog::info("Localization module initialized, map loading deferred");
      }
    } else {
      // Standard global mapping mode
      so_name = glim::Config(glim::GlobalConfig::get_config_path(
                                 "config_global_mapping"))
                    .param<std::string>("global_mapping", "so_name",
                                        "libglobal_mapping.so");

      if (!so_name.empty()) {
        spdlog::info("load {}", so_name);
        global = GlobalMappingBase::load_module(so_name);
        if (global) {
          global_mapping.reset(new AsyncGlobalMapping(global));
        }
      }
    }
  }

  // ROS-related
  using std::placeholders::_1;
  using std::placeholders::_2;
  const std::string imu_topic =
      config_ros.param<std::string>("glim_ros", "imu_topic", "");
  const std::string points_topic =
      config_ros.param<std::string>("glim_ros", "points_topic", "");
  const std::string image_topic =
      config_ros.param<std::string>("glim_ros", "image_topic", "");

  // Subscribers
  rclcpp::SensorDataQoS default_imu_qos;
  default_imu_qos.get_rmw_qos_profile().depth = 1000;
  auto qos =
      get_qos_settings(config_ros, "glim_ros", "imu_qos", default_imu_qos);
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos, std::bind(&GlimROS::imu_callback, this, _1));

  qos        = get_qos_settings(config_ros, "glim_ros", "points_qos");
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, qos, std::bind(&GlimROS::points_callback, this, _1));
#ifdef BUILD_WITH_CV_BRIDGE
  qos       = get_qos_settings(config_ros, "glim_ros", "image_qos");
  image_sub = image_transport::create_subscription(
      this, image_topic, std::bind(&GlimROS::image_callback, this, _1), "raw",
      qos.get_rmw_qos_profile());
#endif

  for (const auto &sub : this->extension_subscriptions()) {
    spdlog::debug("subscribe to {}", sub->topic);
    sub->create_subscriber(*this);
  }

  // Localization mode ROS interfaces
  if (localization_mode) {
    // Subscribe to initial pose topic for manual relocalization
    initial_pose_sub = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&GlimROS::initial_pose_callback, this, _1));

    // Create relocalization service
    relocalization_service = this->create_service<std_srvs::srv::Trigger>(
        "~/trigger_relocalization",
        std::bind(&GlimROS::relocalization_callback, this, _1, _2));

    spdlog::info("Localization mode services initialized");
    spdlog::info("  - Subscribe to /initialpose for manual relocalization");
    spdlog::info("  - Service ~/trigger_relocalization available");
  }

  // Start timer
  timer = this->create_wall_timer(std::chrono::milliseconds(1),
                                  [this]() { timer_callback(); });

  // Register ViewerCallbacks (for extension modules like localization_viewer)
  viewer_user_event_id = ViewerCallbacks::user_event.add(
      std::bind(&GlimROS::on_viewer_user_event, this, _1));
  viewer_load_map_id = ViewerCallbacks::on_load_map.add(
      std::bind(&GlimROS::on_viewer_load_map, this));
  viewer_relocalize_id = ViewerCallbacks::request_relocalize.add(
      std::bind(&GlimROS::on_viewer_request_relocalize, this, _1));

  spdlog::debug("ViewerCallbacks registered");
  spdlog::debug("initialized");
}

GlimROS::~GlimROS() {
  spdlog::debug("quit");

  // Unregister ViewerCallbacks
  if (viewer_user_event_id >= 0) {
    ViewerCallbacks::user_event.remove(viewer_user_event_id);
  }
  if (viewer_load_map_id >= 0) {
    ViewerCallbacks::on_load_map.remove(viewer_load_map_id);
  }
  if (viewer_relocalize_id >= 0) {
    ViewerCallbacks::request_relocalize.remove(viewer_relocalize_id);
  }

  extension_modules.clear();

  if (dump_on_unload) {
    std::string dump_path = "/tmp/dump";
    wait(true);
    save(dump_path);
  }
}

const std::vector<std::shared_ptr<GenericTopicSubscription>> &
GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  const double imu_stamp =
      msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
  const Eigen::Vector3d linear_acc =
      acc_scale * Eigen::Vector3d(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel(msg->angular_velocity.x,
                                    msg->angular_velocity.y,
                                    msg->angular_velocity.z);

  if (!time_keeper->validate_imu_stamp(imu_stamp)) {
    spdlog::warn("skip an invalid IMU data (stamp={})", imu_stamp);
    return;
  }

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  if (sub_mapping) {
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
}

#ifdef BUILD_WITH_CV_BRIDGE
void GlimROS::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  spdlog::trace("image: {}.{}", msg->header.stamp.sec,
                msg->header.stamp.nanosec);

  auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");

  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  odometry_estimation->insert_image(stamp, cv_image->image);
  if (sub_mapping) {
    sub_mapping->insert_image(stamp, cv_image->image);
  }
  if (global_mapping) {
    global_mapping->insert_image(stamp, cv_image->image);
  }
}
#endif

size_t GlimROS::points_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  spdlog::trace("points: {}.{}", msg->header.stamp.sec,
                msg->header.stamp.nanosec);

  auto raw_points = glim::extract_raw_points(*msg, intensity_field);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }

  raw_points->stamp += points_time_offset;
  time_keeper->process(raw_points);
  auto preprocessed = preprocessor->preprocess(raw_points);

  if (keep_raw_points) {
    // note: Raw points are used only in extension modules for visualization
    // purposes.
    //       If you need to reduce the memory footprint, you can safely comment
    //       out the following line.
    preprocessed->raw_points = raw_points;
  }

  odometry_estimation->insert_frame(preprocessed);

  const size_t workload = odometry_estimation->workload();
  spdlog::debug("workload={}", workload);

  return workload;
}

bool GlimROS::needs_wait() {
  for (const auto &ext_module : extension_modules) {
    if (ext_module->needs_wait()) {
      return true;
    }
  }

  return false;
}

void GlimROS::timer_callback() {
  for (const auto &ext_module : extension_modules) {
    if (!ext_module->ok()) {
      rclcpp::shutdown();
    }
  }

  // Load pre-built map after viewer has had time to initialize
  // Wait ~3 seconds (3000 timer ticks at 1ms interval) before loading
  if (localization_mode && !prebuilt_map_loaded && !pending_map_path.empty()) {
    viewer_ready_counter++;

    if (viewer_ready_counter >= 3000) {  // 3 seconds
      spdlog::info("Viewer initialization complete, loading pre-built map...");

      if (global_mapping && global_mapping->load(pending_map_path)) {
        spdlog::info("Successfully loaded pre-built map from: {}", pending_map_path);
        prebuilt_map_loaded = true;
      } else {
        spdlog::error("Failed to load pre-built map from: {}", pending_map_path);
        // Don't retry, just mark as loaded to avoid repeated attempts
        prebuilt_map_loaded = true;
      }
    }
  }

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  // Auto-trigger pending relocalization when frames become available
  if (localization_mode && pending_relocalization && global_mapping) {
    glim::EstimationFrame::ConstPtr latest_frame;
    if (!estimation_frames.empty()) {
      latest_frame = estimation_frames.back();
    } else if (!marginalized_frames.empty()) {
      latest_frame = marginalized_frames.back();
    }

    if (latest_frame) {
      spdlog::info("Auto-triggering pending relocalization");
      global_mapping->relocalize(latest_frame, initial_pose_guess);
      pending_relocalization = false;
      spdlog::info("Relocalization triggered with initial pose: [{}, {}, {}]",
                   initial_pose_guess.translation().x(),
                   initial_pose_guess.translation().y(),
                   initial_pose_guess.translation().z());
    }
  }

  if (sub_mapping) {
    for (const auto &frame : marginalized_frames) {
      sub_mapping->insert_frame(frame);
    }

    auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto &submap : submaps) {
        global_mapping->insert_submap(submap);
      }
    }
  }
}

void GlimROS::wait(bool auto_quit) {
  spdlog::info("waiting for odometry estimation");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto &marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    spdlog::info("waiting for local mapping");
    sub_mapping->join();

    const auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto &submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      spdlog::info("waiting for global mapping");
      global_mapping->join();
    }
  }

  if (!auto_quit) {
    bool terminate = false;
    while (!terminate && rclcpp::ok()) {
      for (const auto &ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

void GlimROS::save(const std::string &path) {
  if (global_mapping)
    global_mapping->save(path);
}

void GlimROS::relocalization_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {

  if (!localization_mode) {
    response->success = false;
    response->message = "Not in localization mode";
    spdlog::warn("Relocalization service called but not in localization mode");
    return;
  }

  if (!global_mapping) {
    response->success = false;
    response->message = "Global mapping not initialized";
    spdlog::error("Relocalization failed: global mapping not initialized");
    return;
  }

  // Get latest estimation frame
  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  // Try estimation frames first, fallback to marginalized frames
  glim::EstimationFrame::ConstPtr latest_frame;
  if (!estimation_frames.empty()) {
    latest_frame = estimation_frames.back();
  } else if (!marginalized_frames.empty()) {
    latest_frame = marginalized_frames.back();
    spdlog::info("Using marginalized frame for relocalization");
  }

  if (!latest_frame) {
    // No frames available yet - mark as pending
    pending_relocalization = true;
    response->success      = true; // Still success, will trigger later
    response->message =
        "No frames available yet. Relocalization will trigger automatically "
        "when data arrives.";
    spdlog::warn(
        "No estimation frames available yet. Relocalization marked as "
        "pending.");
    return;
  }

  spdlog::info("Triggering relocalization with initial pose:");
  spdlog::info("  Position: [{}, {}, {}]", initial_pose_guess.translation().x(),
               initial_pose_guess.translation().y(),
               initial_pose_guess.translation().z());

  // Trigger relocalization in global mapping
  global_mapping->relocalize(latest_frame, initial_pose_guess);
  pending_relocalization = false;

  response->success = true;
  response->message = "Relocalization triggered successfully";
  spdlog::info("Relocalization triggered");
}

void GlimROS::initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

  if (!localization_mode) {
    spdlog::warn("Received initial pose but not in localization mode");
    return;
  }

  // Convert ROS pose to Eigen::Isometry3d
  initial_pose_guess = Eigen::Isometry3d::Identity();
  initial_pose_guess.translation() =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);

  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  initial_pose_guess.linear() = q.toRotationMatrix();

  spdlog::info("Received initial pose from /initialpose:");
  spdlog::info("  Position: [{}, {}, {}]", initial_pose_guess.translation().x(),
               initial_pose_guess.translation().y(),
               initial_pose_guess.translation().z());

  // Automatically trigger relocalization when initial pose is set
  if (global_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_frames, marginalized_frames);

    // Try estimation frames first, fallback to marginalized frames
    glim::EstimationFrame::ConstPtr latest_frame;
    if (!estimation_frames.empty()) {
      latest_frame = estimation_frames.back();
    } else if (!marginalized_frames.empty()) {
      latest_frame = marginalized_frames.back();
      spdlog::info("Using marginalized frame for relocalization");
    }

    if (latest_frame) {
      global_mapping->relocalize(latest_frame, initial_pose_guess);
      pending_relocalization = false;
      spdlog::info("Auto-triggered relocalization from /initialpose");
    } else {
      pending_relocalization = true;
      spdlog::warn(
          "Initial pose received but no estimation frames available yet");
      spdlog::warn(
          "Relocalization will trigger automatically when sensor data arrives");
    }
  }
}

// ============================================================
// ViewerCallbacks Handlers
// ============================================================

void GlimROS::on_viewer_user_event(int pose_id) {
  spdlog::info("ViewerCallback: user_event triggered with pose_id={}", pose_id);

  // This callback is triggered when user clicks "Loc Save Pose" button in
  // viewer pose_id ranges from 0-9 indicating which pose slot to save

  if (!localization_mode) {
    spdlog::warn("User event received but not in localization mode");
    return;
  }

  // Get current estimated pose
  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  if (estimation_frames.empty()) {
    spdlog::warn("No estimation frames available to save");
    return;
  }

  auto                    latest_frame = estimation_frames.back();
  const Eigen::Isometry3d pose         = latest_frame->T_world_sensor();

  spdlog::info("Saved pose {} at position: [{}, {}, {}]", pose_id,
               pose.translation().x(), pose.translation().y(),
               pose.translation().z());

  // TODO: You can extend this to:
  // - Publish saved poses to a ROS topic
  // - Store poses for later retrieval
  // - Create markers for visualization
}

void GlimROS::on_viewer_load_map() {
  spdlog::info("ViewerCallback: on_load_map triggered");

  // This callback is triggered when user clicks "Load Map" button in viewer

  if (!localization_mode) {
    spdlog::warn("Load map requested but not in localization mode");
    return;
  }

  // In localization mode, map is already loaded during initialization
  // This callback could be used to:
  // - Reload the map
  // - Load a different map
  // - Refresh map visualization

  spdlog::info("Map already loaded in localization mode");

  // TODO: You can extend this to:
  // - Add a ROS service to dynamically load maps
  // - Reload the current map
  // - Switch between different maps
}

void GlimROS::on_viewer_request_relocalize(const Eigen::Vector3d &pos) {
  spdlog::info(
      "ViewerCallback: request_relocalize triggered at position: [{}, {}, {}]",
      pos.x(), pos.y(), pos.z());

  // This callback is triggered when user right-clicks on a point in the viewer
  // and selects "Relocalize here"

  if (!localization_mode) {
    spdlog::warn("Relocalization requested but not in localization mode");
    return;
  }

  if (!global_mapping) {
    spdlog::error("Global mapping not initialized");
    return;
  }

  // Create initial pose guess from clicked position
  // Use identity rotation (could be improved with more sophisticated guessing)
  Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
  initial_pose.translation()     = pos;

  // Update the stored initial pose guess
  initial_pose_guess = initial_pose;

  spdlog::info("Initial pose set from viewer at position: [{}, {}, {}]",
               pos.x(), pos.y(), pos.z());

  // Get latest estimation frame
  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  // Try to use estimation frames first, fallback to marginalized frames
  glim::EstimationFrame::ConstPtr latest_frame;
  if (!estimation_frames.empty()) {
    latest_frame = estimation_frames.back();
  } else if (!marginalized_frames.empty()) {
    latest_frame = marginalized_frames.back();
    spdlog::info("Using marginalized frame for relocalization");
  }

  if (latest_frame) {
    // Trigger relocalization immediately if we have frames
    global_mapping->relocalize(latest_frame, initial_pose);
    pending_relocalization = false;
    spdlog::info(
        "Relocalization triggered from viewer at position: [{}, {}, {}]",
        pos.x(), pos.y(), pos.z());
  } else {
    // No frames available yet - will auto-trigger when first frame arrives
    pending_relocalization = true;
    spdlog::warn("No estimation frames available yet. Initial pose saved.");
    spdlog::warn(
        "Relocalization will be triggered automatically when sensor data "
        "arrives.");
  }
}

} // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimROS);