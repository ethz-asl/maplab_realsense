#include "maplab-realsense/zr300.h"

#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <librealsense/rs.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

namespace maplab_realsense {

RealSenseConfiguration RealSenseConfiguration::getFromRosParams(
    const ros::NodeHandle& private_nh) {
  RealSenseConfiguration config;

  private_nh.param("imu/enabled", config.imu_enabled, config.imu_enabled);

  private_nh.param(
      "fisheye/enabled", config.fisheye_enabled, config.fisheye_enabled);
  if (config.fisheye_enabled) {
    private_nh.param(
        "fisheye/width", config.fisheye_width, config.fisheye_width);
    private_nh.param(
        "fisheye/height", config.fisheye_height, config.fisheye_height);
    private_nh.param("fisheye/fps", config.fisheye_fps, config.fisheye_fps);
    private_nh.param(
        "fisheye/enable_auto_exposure", config.fisheye_enable_auto_exposure,
        config.fisheye_enable_auto_exposure);
    private_nh.param(
        "fisheye/exposure_ms", config.fisheye_exposure_ms,
        config.fisheye_exposure_ms);
    private_nh.param("fisheye/gain", config.fisheye_gain, config.fisheye_gain);
    private_nh.param(
        "fisheye/subsample_factor", config.fisheye_subsample_factor,
        config.fisheye_subsample_factor);
  }

  // Depth/IR/pointcloud config.
  private_nh.param("depth/enabled", config.depth_enabled, config.depth_enabled);
  private_nh.param(
      "depth/enable_pointcloud", config.pointcloud_enabled,
      config.pointcloud_enabled);
  private_nh.param(
      "infrared/enabled", config.infrared_enabled, config.infrared_enabled);
  if (config.depth_enabled || config.pointcloud_enabled ||
      config.infrared_enabled) {
    private_nh.param("depth/width", config.depth_width, config.depth_width);
    private_nh.param("depth/height", config.depth_height, config.depth_height);
    private_nh.param("depth/fps", config.depth_fps, 30);
    private_nh.param(
        "depth/subsample_factor", config.depth_subsample_factor,
        config.depth_subsample_factor);
    private_nh.param(
        "depth/median_filter_enabled", config.depth_median_filter_enabled,
        config.depth_median_filter_enabled);
    private_nh.param(
        "depth/min_max_filter_enabled", config.depth_min_max_filter_enabled,
        config.depth_min_max_filter_enabled);
    private_nh.param(
        "depth/min_max_filter_size", config.depth_min_max_filter_size,
        config.depth_min_max_filter_size);
    private_nh.param(
        "depth/min_max_filter_threshold", config.depth_min_max_filter_threshold,
        config.depth_min_max_filter_threshold);

    if (config.infrared_enabled) {
      private_nh.param(
          "infrared/subsample_factor", config.infrared_subsample_factor,
          config.infrared_subsample_factor);
    }

    if (config.pointcloud_enabled) {
      private_nh.param(
          "pointcloud/min_h", config.pointcloud_hsv_min_h,
          config.pointcloud_hsv_min_h);
      private_nh.param(
          "pointcloud/min_s", config.pointcloud_hsv_min_s,
          config.pointcloud_hsv_min_s);
      private_nh.param(
          "pointcloud/min_v", config.pointcloud_hsv_min_v,
          config.pointcloud_hsv_min_v);

      private_nh.param(
          "pointcloud/max_h", config.pointcloud_hsv_max_h,
          config.pointcloud_hsv_max_h);
      private_nh.param(
          "pointcloud/max_s", config.pointcloud_hsv_max_s,
          config.pointcloud_hsv_max_s);
      private_nh.param(
          "pointcloud/max_v", config.pointcloud_hsv_max_v,
          config.pointcloud_hsv_max_v);
      private_nh.param(
          "pointcloud/color_filter_enabled",
          config.pointcloud_color_filter_enabled,
          config.pointcloud_color_filter_enabled);
    }
  }

  private_nh.param("color/enabled", config.color_enabled, config.color_enabled);
  if (config.color_enabled || config.pointcloud_enabled) {
    private_nh.param("color/width", config.color_width, config.color_width);
    private_nh.param("color/height", config.color_height, config.color_height);
    private_nh.param("color/fps", config.color_fps, config.color_fps);
    private_nh.param(
        "color/subsample_factor", config.color_subsample_factor,
        config.color_subsample_factor);
  }

  return config;
}

const std::string ZR300::kFisheyeTopic = "fisheye";
const std::string ZR300::kColorTopic = "color";
const std::string ZR300::kImuTopic = "imu";
const std::string ZR300::kInfraredTopic = "ir_1";
const std::string ZR300::kInfrared2Topic = "ir_2";
const std::string ZR300::kDepthTopic = "depth";
const std::string ZR300::kPointCloudTopic = "pointcloud";

ZR300::ZR300(
    ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string& frameId)
    : nh_(nh), private_nh_(private_nh), gyro_measurement_index_(0u) {
  config_ = RealSenseConfiguration::getFromRosParams(private_nh);

  initializePublishers(&nh);

  device_time_translator_.reset(
      new cuckoo_time_translator::UnwrappedDeviceTimeTranslator(
          cuckoo_time_translator::ClockParameters(kSecondsToNanoseconds),
          private_nh_.getNamespace(),
          cuckoo_time_translator::Defaults().setFilterAlgorithm(
              cuckoo_time_translator::FilterAlgorithm::ConvexHull)));
}

bool ZR300::start() {
  rs::log_to_console(rs::log_severity::info);

  LOG(INFO) << "Found " << zr300_context_.get_device_count()
            << " RealSense devices.";

  if (zr300_context_.get_device_count() == 0) {
    LOG(ERROR) << "No RealSense devices found.";
    return EXIT_FAILURE;
  }

  zr300_device_ = zr300_context_.get_device(0);
  LOG(INFO) << " Device " << zr300_device_->get_name();
  LOG(INFO) << " Serial number: " << zr300_device_->get_serial();

  if (zr300_device_->supports(rs::capabilities::motion_events)) {
    zr300_device_->enable_motion_tracking(
        std::bind(&ZR300::motionCallback, this, std::placeholders::_1));
  } else {
    LOG(FATAL) << "This sensor does not support motion tracking mode!";
    return false;
  }

  enableSensorStreams();
  configureStaticOptions();
  registerCallbacks();

  zr300_device_->start(rs::source::all_sources);

  return true;
}

void ZR300::stop() {
  zr300_device_->stop(rs::source::all_sources);
  zr300_device_->disable_motion_tracking();
}

void ZR300::initializePublishers(ros::NodeHandle* nh) {
  CHECK_NOTNULL(nh);

  auto advertiseCamera =
      [nh](const std::string& name) -> image_transport::Publisher {
    ros::NodeHandle _nh(*nh, name);
    image_transport::ImageTransport it(_nh);
    return it.advertise(name, 1);
  };

  if (config_.fisheye_enabled) {
    fisheye_publisher_ = advertiseCamera(kFisheyeTopic);
  }
  if (config_.color_enabled) {
    color_publisher_ = advertiseCamera(kColorTopic);
  }
  if (config_.infrared_enabled) {
    infrared_publisher_ = advertiseCamera(kInfraredTopic);
    infrared_2_publisher_ = advertiseCamera(kInfrared2Topic);
  }
  if (config_.depth_enabled) {
    depth_publisher_ = advertiseCamera(kDepthTopic);
  }

  if (config_.imu_enabled) {
    imu_publisher_ = nh->advertise<sensor_msgs::Imu>(kImuTopic, 1);
  }

  if (config_.pointcloud_enabled) {
    pointcloud_publisher_ =
        nh->advertise<sensor_msgs::PointCloud2>(kPointCloudTopic, 1);
  }
}

void ZR300::registerCallbacks() {
  if (config_.fisheye_enabled) {
    zr300_device_->set_frame_callback(
        rs::stream::fisheye,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }
  if (config_.color_enabled) {
    zr300_device_->set_frame_callback(
        rs::stream::color,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }
  if (config_.infrared_enabled) {
    zr300_device_->set_frame_callback(
        rs::stream::infrared,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
    zr300_device_->set_frame_callback(
        rs::stream::infrared2,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }

  if (config_.depth_enabled) {
    zr300_device_->set_frame_callback(
        rs::stream::depth,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }

  if (config_.pointcloud_enabled) {
    // TODO(mfehr): fix this, rs::stream::points is not a "native" stream,
    // probably needs a different callback registration.
    // zr300_device_->set_frame_callback(
    //     rs::stream::points,
    //     std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }
}

void ZR300::configureStaticOptions() {
  // This option is necessary to make sure the data is properly synchronized
  // inside the sensor. Keep it turned on.
  zr300_device_->set_option(rs::option::fisheye_strobe, 1);

  zr300_device_->set_option(rs::option::fisheye_gain, config_.fisheye_gain);
  zr300_device_->set_option(
      rs::option::fisheye_exposure, config_.fisheye_exposure_ms);
  zr300_device_->set_option(
      rs::option::fisheye_color_auto_exposure,
      config_.fisheye_enable_auto_exposure);
  zr300_device_->set_option(rs::option::fisheye_color_auto_exposure_mode, 2);
  // Flicker rate of ambient light.
  zr300_device_->set_option(rs::option::fisheye_color_auto_exposure_rate, 50);
}

void ZR300::enableSensorStreams() {
  if (config_.fisheye_enabled) {
    zr300_device_->enable_stream(
        rs::stream::fisheye, config_.fisheye_width, config_.fisheye_height,
        rs::format::raw8, config_.fisheye_fps);
  }
  if (config_.depth_enabled || config_.pointcloud_enabled) {
    zr300_device_->enable_stream(
        rs::stream::depth, config_.depth_width, config_.depth_height,
        rs::format::z16, config_.depth_fps);
  }
  if (config_.infrared_enabled) {
    zr300_device_->enable_stream(
        rs::stream::infrared, config_.depth_width, config_.depth_height,
        rs::format::y8, config_.depth_fps);
    zr300_device_->enable_stream(
        rs::stream::infrared2, config_.depth_width, config_.depth_height,
        rs::format::y8, config_.depth_fps);
  }
  if (config_.color_enabled || config_.pointcloud_enabled) {
    zr300_device_->enable_stream(
        rs::stream::color, config_.color_width, config_.color_height,
        rs::format::rgb8, config_.color_fps);
  }
}

void ZR300::frameCallback(const rs::frame& frame) {
  CHECK_GE(frame.get_frame_number(), 0u);

  const rs::stream stream_type = frame.get_stream_type();
  const size_t frame_number = static_cast<size_t>(frame.get_frame_number());
  const double sensor_timestamp_s = static_cast<double>(frame.get_timestamp());

  VLOG(100) << "Frame data:"
            << "\n\tROS time: " << ros::Time::now()
            << "\n\tSensor timestamp: " << std::setprecision(8)
            << frame.get_timestamp() << "\n\tsource: " << std::setw(13)
            << frame.get_stream_type() << "\n\tframe_num: " << frame_number
            << "\n\tdomain: "
            << static_cast<int>(frame.get_frame_timestamp_domain());

  switch (stream_type) {
    case rs::stream::fisheye: {
      if (fisheye_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, config_.fisheye_fps * 20)
            << "No subscribers for the fisheye images found!";
        return;
      }

      if (frame.get_frame_timestamp_domain() !=
          rs::timestamp_domain::microcontroller) {
        LOG(ERROR) << "The timestamp of frame " << frame.get_frame_number()
                   << " of type " << stream_type
                   << " did not originate from the motion tracking "
                   << "microcontroller. The timestamp domain is set to: "
                   << static_cast<int>(frame.get_frame_timestamp_domain())
                   << " (camera = 0, microcontroller = 1). Skipping frame!";
        return;
      }

      // Check if timestamp is strictly monotonically increasing.
      CHECK_GT(sensor_timestamp_s, last_fisheye_frame_timestamp_s_);
      last_fisheye_frame_timestamp_s_ = sensor_timestamp_s;

      // Compose image message.
      sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

      CHECK(device_time_translator_->isReadyToTranslate());
      msg->header.stamp = device_time_translator_->translate(
          sensor_timestamp_s * kMillisecondsToNanoseconds);

      msg->header.frame_id = "fisheye";

      msg->height = frame.get_height();
      msg->width = frame.get_width();
      msg->step = msg->width;
      msg->encoding = sensor_msgs::image_encodings::MONO8;
      const size_t size = msg->height * msg->step;
      msg->data.resize(size);
      memcpy(msg->data.data(), frame.get_data(), size);

      // Publish image message.
      fisheye_publisher_.publish(msg);
      break;
    }
    case rs::stream::color: {
      if (color_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, config_.color_fps * 20)
            << "No subscribers for the color images found!";
        return;
      }

      // Check if timestamp is strictly monotonically increasing.
      CHECK_GT(sensor_timestamp_s, last_color_frame_timestamp_s_);
      last_color_frame_timestamp_s_ = sensor_timestamp_s;

      // Compose image message.
      sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

      CHECK(device_time_translator_->isReadyToTranslate());
      msg->header.stamp = device_time_translator_->translate(
          sensor_timestamp_s * kMillisecondsToNanoseconds);

      msg->header.frame_id = "color";

      msg->height = frame.get_height();
      msg->width = frame.get_width();
      msg->step = msg->width * 3;
      msg->encoding = sensor_msgs::image_encodings::RGB8;
      const size_t size = msg->height * msg->step;
      msg->data.resize(size);
      memcpy(msg->data.data(), frame.get_data(), size);

      // Compose camera info message.
      // sensor_msgs::CameraInfoPtr camera_info_msg =
      //     boost::make_shared<sensor_msgs::CameraInfo>(infoColor_);
      // camera_info_msg->header = msg->header;

      // Publish image.
      color_publisher_.publish(msg);

      break;
    }
    case rs::stream::infrared2:
      if (infrared_2_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, 300)
            << "No subscribers for the infrared images found!";
        return;
      }
    // Fall through intended.
    case rs::stream::infrared: {
      if (infrared_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, config_.depth_fps * 20)
            << "No subscribers for the infrared 2 images found!";
        return;
      }

      // Compose image message.
      sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

      CHECK(device_time_translator_->isReadyToTranslate());
      msg->header.stamp = device_time_translator_->translate(
          sensor_timestamp_s * kMillisecondsToNanoseconds);

      msg->height = frame.get_height();
      msg->width = frame.get_width();
      msg->step = msg->width;
      msg->encoding = sensor_msgs::image_encodings::MONO8;
      const size_t size = msg->height * msg->step;
      msg->data.resize(size);
      memcpy(msg->data.data(), frame.get_data(), size);

      // Publish image.
      if (stream_type == rs::stream::infrared) {
        // Check if timestamp is strictly monotonically increasing.
        CHECK_GT(sensor_timestamp_s, last_infrared_frame_timestamp_s_);
        last_infrared_frame_timestamp_s_ = sensor_timestamp_s;

        msg->header.frame_id = "infrared";
        infrared_publisher_.publish(msg);
      } else if (stream_type == rs::stream::infrared2) {
        // Check if timestamp is strictly monotonically increasing.
        CHECK_GT(sensor_timestamp_s, last_infrared_2_frame_timestamp_s_);
        last_infrared_2_frame_timestamp_s_ = sensor_timestamp_s;

        msg->header.frame_id = "infrared_2";
        infrared_2_publisher_.publish(msg);
      }
      break;
    }
    case rs::stream::depth: {
      if (depth_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, config_.depth_fps * 20)
            << "No subscribers for the depth maps found!";
        return;
      }

      // Check if timestamp is strictly monotonically increasing.
      CHECK_GT(sensor_timestamp_s, last_depth_frame_timestamp_s_);
      last_depth_frame_timestamp_s_ = sensor_timestamp_s;

      // Retrieve and buffer depth map.
      CHECK_GT(frame.get_frame_number(), latest_depth_frame_number_);
      latest_depth_frame_number_ = frame.get_frame_number();
      latest_depth_map_ =
          cv::Mat(frame.get_height(), frame.get_width(), CV_16UC1);
      memcpy(
          latest_depth_map_.data, frame.get_data(),
          latest_depth_map_.total() * latest_depth_map_.elemSize());

      // Compose image message.
      sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

      CHECK(device_time_translator_->isReadyToTranslate());
      msg->header.stamp = device_time_translator_->translate(
          sensor_timestamp_s * kMillisecondsToNanoseconds);

      msg->header.frame_id = "depth";

      msg->height = frame.get_height();
      msg->width = frame.get_width();
      msg->step = msg->width * 2;
      msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      const size_t size = msg->height * msg->step;
      msg->data.resize(size);
      memcpy(msg->data.data(), frame.get_data(), size);

      // Publish image.
      depth_publisher_.publish(msg);

      break;
    }

    default:
      LOG(FATAL) << "Unknown frame type: " << static_cast<int>(stream_type);
  }
}

void ZR300::motionCallback(const rs::motion_data& entry) {
  const rs::event motion_event =
      static_cast<rs::event>(entry.timestamp_data.source_id);
  const double motion_timestamp =
      static_cast<double>(entry.timestamp_data.timestamp);

  VLOG(100) << "Motion " << motion_event << " " << motion_timestamp << " "
            << entry.timestamp_data.frame_number;

  switch (motion_event) {
    case rs::event::event_imu_gyro: {
      // Provide a time update to the time translator.
      const ros::Time time_now = ros::Time::now();
      device_time_translator_->update(
          static_cast<std::uint64_t>(
              entry.timestamp_data.timestamp * kMillisecondsToNanoseconds),
          time_now);

      ++gyro_measurement_index_;
      if (gyro_measurement_index_ < kSkipNFirstGyroMeasurements) {
        return;
      }

      std::vector<ImuSynchronizer::ImuData> imu_data;
      imu_synchronizer_.registerGyroMeasurement(
          motion_timestamp,
          Eigen::Vector3d(entry.axes[0], entry.axes[1], entry.axes[2]),
          &imu_data);

      if (imu_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, 2000) << "No subscribers for the IMU data found!";
        return;
      }

      for (const ImuSynchronizer::ImuData& item : imu_data) {
        sensor_msgs::ImuPtr msg = boost::make_shared<sensor_msgs::Imu>();

        msg->header.stamp = device_time_translator_->translate(
            item.timestamp * kMillisecondsToNanoseconds);

        msg->header.seq = entry.timestamp_data.frame_number;
        msg->header.frame_id = "imu";

        msg->angular_velocity.x = item.gyro[0];
        msg->angular_velocity.y = item.gyro[1];
        msg->angular_velocity.z = item.gyro[2];

        msg->linear_acceleration.x = item.acc[0];
        msg->linear_acceleration.y = item.acc[1];
        msg->linear_acceleration.z = item.acc[2];

        msg->orientation_covariance[0] = -1.0;  // No orientation estimate.

        imu_publisher_.publish(msg);
      }
      break;
    }
    case rs::event::event_imu_accel: {
      imu_synchronizer_.registerAccelerometerMeasurement(
          motion_timestamp,
          Eigen::Vector3d(entry.axes[0], entry.axes[1], entry.axes[2]));

      break;
    }
    default:
      LOG(FATAL) << "Unknown motion tracking event: "
                 << static_cast<int>(entry.timestamp_data.source_id);
  }
}
}  // namespace maplab_realsense
