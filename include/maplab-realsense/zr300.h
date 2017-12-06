#ifndef MAPLAB_REALSENSE_ZR300_H_
#define MAPLAB_REALSENSE_ZR300_H_

#include <string>

#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/publisher.h>
#include <librealsense/rs.hpp>
#include <librealsense/rsutil.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
  struct rgb_color {
    uint8_t r, g, b;
  };

  void motionCallback(const rs::motion_data& entry);
  void frameCallback(const rs::frame& frame);

  void initializePublishers(ros::NodeHandle* nh);
  void enableSensorStreams();
  void configureStaticOptions();
  void registerCallbacks();
  void retrieveCameraCalibrations();
  void publishStaticTransforms();
  void publishPointCloudIfDataAvailable();

  void improveDepth(cv::Mat* depth_image);

  void convertCalibrationToCameraInfoMsg(
      const rs::intrinsics& intrinsics, const rs::extrinsics& extrinsics,
      sensor_msgs::CameraInfo* camera_info);
  geometry_msgs::TransformStamped convertExtrinsicsToTf(
      const rs::extrinsics& T_to_from, const ros::Time& stamp,
      const std::string& parent, const std::string& child);
  void depthToPointcloud(
      const cv::Mat& rgb_color_image, const cv::Mat& depth_image,
      pcl::PointCloud<pcl::PointXYZRGB>* pointcloud);
  void invertExtrinsics(const rs::extrinsics& T_A_B_in, rs::extrinsics* T_B_A);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ZR300Config config_;

  image_transport::CameraPublisher fisheye_publisher_;
  image_transport::CameraPublisher color_publisher_;
  image_transport::CameraPublisher infrared_publisher_;
  image_transport::CameraPublisher infrared_2_publisher_;
  image_transport::CameraPublisher depth_publisher_;
  ros::Publisher imu_publisher_;
  ros::Publisher pointcloud_publisher_;

  rs::extrinsics T_infrared_fisheye_;
  sensor_msgs::CameraInfo fisheye_camera_info_;
  rs::extrinsics T_infrared_depth_;
  sensor_msgs::CameraInfo depth_camera_info_;
  rs::extrinsics T_infrared_infrared_;
  rs::intrinsics intrinsics_depth_;
  sensor_msgs::CameraInfo infrared_camera_info_;
  rs::extrinsics T_infrared_infrared_2_;
  sensor_msgs::CameraInfo infrared_2_camera_info_;
  rs::extrinsics T_infrared_color_;
  rs::extrinsics T_color_infrared_;
  sensor_msgs::CameraInfo color_camera_info_;
  rs::intrinsics intrinsics_color_;

  tf2_ros::StaticTransformBroadcaster extrinsics_broadcaster_;

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
  size_t latest_color_frame_number_ = 0u;
  cv::Mat latest_color_image_;
  size_t latest_point_cloud_frame_number_ = 0u;

  std::unique_ptr<cuckoo_time_translator::UnwrappedDeviceTimeTranslator>
      device_time_translator_;

  size_t angular_velocity_index_;
};

}  // namespace maplab_realsense

#endif  // MAPLAB_REALSENSE_ZR300_H_
