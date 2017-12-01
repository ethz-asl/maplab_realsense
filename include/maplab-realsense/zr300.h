#ifndef MAPLAB_REALSENSE_ZR300_H_
#define MAPLAB_REALSENSE_ZR300_H_

#include <string>

#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <image_transport/publisher.h>
#include <librealsense/rs.hpp>
#include <ros/ros.h>

#include "maplab-realsense/time-synchronizer.h"

namespace maplab_realsense {

struct RealSenseConfiguration {
  bool fisheye_enable_auto_exposure_ = true;
  double fisheye_exposure_ms_ = 25.;
  double fisheye_gain_ = 9.;
  int fisheye_subsample_factor_ = 1.;

  int fisheye_width = 640;
  int fisheye_height = 480;
  int fisheye_fps = 30;

  int color_width = 640;
  int color_height = 480;
  int color_fps = 30;
};

class ZR300 {
 public:
  ZR300(ros::NodeHandle nh, ros::NodeHandle private_nh,
        const std::string& frameId = "");

  bool start();
  void stop();

 private:
  void motionCallback(const rs::motion_data& entry);
  void timestampCallback(const rs::timestamp_data& entry);
  void frameCallback(const rs::frame& frame);

  void enableCameraStreams();
  void configureStaticOptions();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  image_transport::Publisher fisheye_publisher_;
  image_transport::Publisher color_publisher_;
  ros::Publisher imu_publisher_;

  rs::context zr300_context_;
  rs::device* zr300_device_;

  static const std::string kFisheyeTopic;
  static const std::string kColorTopic;
  static const std::string kImuTopic;

  static constexpr double kImuTimestampToSeconds = 1e-3;

  static constexpr int kSecondsToNanoseconds = 1e9;
  static constexpr int kMillisecondsToNanoseconds = 1e6;

  RealSenseConfiguration realsense_config_;

  ImuSynchronizer imu_synchronizer_;
  FrameTimestampSynchronizer frame_timestamp_synchronizer_;

  std::unique_ptr<cuckoo_time_translator::UnwrappedDeviceTimeTranslator> device_time_translator_;

  size_t gyro_measurement_index_;
  static constexpr size_t kSkipNFirstGyroMeasurements = 100u;
};

}  // namespace maplab_realsense

#endif  // MAPLAB_REALSENSE_ZR300_H_
