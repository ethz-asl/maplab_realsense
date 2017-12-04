#include "maplab-realsense/zr300.h"

#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <librealsense/rs.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

namespace maplab_realsense {

const std::string ZR300::kFisheyeTopic = "fisheye";
const std::string ZR300::kColorTopic = "color";
const std::string ZR300::kImuTopic = "imu";

ZR300::ZR300(
    ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string& frameId)
    : nh_(nh), private_nh_(private_nh), gyro_measurement_index_(0u) {
  auto advertiseCamera =
      [nh](const std::string& name) -> image_transport::Publisher {
    ros::NodeHandle _nh(nh, name);
    image_transport::ImageTransport it(_nh);
    return it.advertise(name, 1);
  };

  color_publisher_ = advertiseCamera(kColorTopic);
  fisheye_publisher_ = advertiseCamera(kFisheyeTopic);
  imu_publisher_ = nh.advertise<sensor_msgs::Imu>(kImuTopic, 1);

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
        std::bind(&ZR300::motionCallback, this, std::placeholders::_1),
        std::bind(&ZR300::timestampCallback, this, std::placeholders::_1));
  } else {
    ROS_ERROR("motion module not supported");
    return false;
  }

  enableCameraStreams();
  configureStaticOptions();

  zr300_device_->set_frame_callback(
      rs::stream::fisheye,
      std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  zr300_device_->start(rs::source::all_sources);

  return true;
}

void ZR300::stop() {
  zr300_device_->stop(rs::source::all_sources);
  zr300_device_->disable_motion_tracking();
}

void ZR300::configureStaticOptions() {
  // This option is necessary to make sure the data is properly synchronized
  // inside the sensor. Keep it turned on.
  zr300_device_->set_option(rs::option::fisheye_strobe, 1);

  zr300_device_->set_option(
      rs::option::fisheye_color_auto_exposure,
      realsense_config_.fisheye_enable_auto_exposure_);
  zr300_device_->set_option(
      rs::option::fisheye_gain, realsense_config_.fisheye_gain_);
  zr300_device_->set_option(
      rs::option::fisheye_exposure, realsense_config_.fisheye_exposure_ms_);
  zr300_device_->set_option(rs::option::fisheye_color_auto_exposure_mode, 2);

  // Flicker rate of ambient light.
  zr300_device_->set_option(rs::option::fisheye_color_auto_exposure_rate, 50);
}

void ZR300::enableCameraStreams() {
  zr300_device_->enable_stream(
      rs::stream::fisheye, realsense_config_.fisheye_width,
      realsense_config_.fisheye_height, rs::format::raw8,
      realsense_config_.fisheye_fps);
  zr300_device_->enable_stream(
      rs::stream::color, realsense_config_.color_width,
      realsense_config_.color_height, rs::format::rgb8,
      realsense_config_.color_fps);
}

void ZR300::frameCallback(const rs::frame& frame) {
  CHECK_GE(frame.get_frame_number(), 0u);

  const rs::stream stream_type = frame.get_stream_type();
  const size_t frame_number = static_cast<size_t>(frame.get_frame_number());

  VLOG(100) << "Frame " << stream_type << " " << frame_number;

  switch (stream_type) {
    case rs::stream::fisheye: {
      if (frame.get_frame_timestamp_domain() !=
          rs::timestamp_domain::microcontroller) {
        LOG(ERROR) << "Timestamp of frame " << frame.get_frame_number()
                   << " might be corrupt. Skipping....";
        return;
      }

      const double hw_timestamp =
          frame_timestamp_synchronizer_.getTimestampForFrame(
              frame.get_frame_number());

      sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

      CHECK(device_time_translator_->isReadyToTranslate());
      msg->header.stamp = device_time_translator_->translate(
          hw_timestamp * kMillisecondsToNanoseconds);

      msg->header.frame_id = "fisheye";

      msg->height = frame.get_height();
      msg->width = frame.get_width();
      msg->step = msg->width;
      msg->encoding = sensor_msgs::image_encodings::MONO8;
      const size_t size = msg->height * msg->step;
      msg->data.resize(size);
      memcpy(msg->data.data(), frame.get_data(), size);
      fisheye_publisher_.publish(msg);
      break;
    }
    case rs::stream::color: {
      LOG(WARNING) << "Received color frame: TODO(mfehr): Implement publisher!";
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

void ZR300::timestampCallback(const rs::timestamp_data& entry) {
  const rs::event timestamp_event = static_cast<rs::event>(entry.source_id);
  if (timestamp_event == rs::event::event_imu_motion_cam) {
    frame_timestamp_synchronizer_.registerTimestamp(
        entry.frame_number, (double)entry.timestamp);
  }
}

}  // namespace maplab_realsense
