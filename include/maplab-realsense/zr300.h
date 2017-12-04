#ifndef MAPLAB_REALSENSE_ZR300_H_
#define MAPLAB_REALSENSE_ZR300_H_

#include <string>

#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <image_transport/publisher.h>
#include <librealsense/rs.hpp>
#include <librealsense/rsutil.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include "maplab-realsense/time-synchronizer.h"
#include "maplab-realsense/zr300-config.h"

namespace maplab_realsense {

class ZR300 {
 public:
  ZR300(
      ros::NodeHandle nh, ros::NodeHandle private_nh,
      const std::string& frameId = "");

  bool start();
  void stop();

 private:
  void motionCallback(const rs::motion_data& entry);
  void frameCallback(const rs::frame& frame);

  void initializePublishers(ros::NodeHandle* nh);
  void enableSensorStreams();
  void configureStaticOptions();
  void registerCallbacks();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  RealSenseConfiguration config_;

  image_transport::Publisher fisheye_publisher_;
  image_transport::Publisher color_publisher_;
  image_transport::Publisher infrared_publisher_;
  image_transport::Publisher infrared_2_publisher_;
  image_transport::Publisher depth_publisher_;
  ros::Publisher imu_publisher_;
  ros::Publisher pointcloud_publisher_;

  rs::context zr300_context_;
  rs::device* zr300_device_;

  static constexpr double kImuTimestampToSeconds = 1e-3;
  static constexpr int kSecondsToNanoseconds = 1e9;
  static constexpr int kMillisecondsToNanoseconds = 1e6;

  ImuSynchronizer imu_synchronizer_;

  double last_color_frame_timestamp_s_ = -1.0;
  double last_fisheye_frame_timestamp_s_ = -1.0;
  double last_infrared_frame_timestamp_s_ = -1.0;
  double last_infrared_2_frame_timestamp_s_ = -1.0;
  double last_depth_frame_timestamp_s_ = -1.0;

  size_t latest_depth_frame_number_ = 0u;
  cv::Mat latest_depth_map_;

  std::unique_ptr<cuckoo_time_translator::UnwrappedDeviceTimeTranslator>
      device_time_translator_;

  size_t gyro_measurement_index_;
};

}  // namespace maplab_realsense

#endif  // MAPLAB_REALSENSE_ZR300_H_
