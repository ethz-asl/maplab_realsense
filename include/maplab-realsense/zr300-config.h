#ifndef MAPLAB_REALSENSE_ZR300_CONFIG_H_
#define MAPLAB_REALSENSE_ZR300_CONFIG_H_

#include <librealsense/rs.hpp>
#include <librealsense/rsutil.h>
#include <ros/ros.h>

namespace maplab_realsense {

struct ZR300Config {
  // ROS config.
  static const std::string kFisheyeTopic;
  static const std::string kColorTopic;
  static const std::string kImuTopic;
  static const std::string kInfraredTopic;
  static const std::string kInfrared2Topic;
  static const std::string kDepthTopic;
  static const std::string kPointCloudTopic;

  static const std::string kImageSuffix;

  // Imu config.
  bool imu_enabled = true;
  int imu_skip_first_n_gyro_measurements = 100;

  // Fisheye config.
  bool fisheye_enabled = true;
  bool fisheye_enable_auto_exposure = true;
  // 40ms is minimum
  double fisheye_exposure_ms = 25.;
  double fisheye_gain = 9.;
  int fisheye_subsample_factor = 1;
  int fisheye_width = 640;
  int fisheye_height = 480;
  int fisheye_fps = 30;

  // Color config.
  bool color_enabled = true;
  int color_width = 640;
  int color_height = 480;
  int color_fps = 30;
  int color_subsample_factor = 1;

  // Depth config.
  bool depth_enabled = true;
  int depth_width = 640;
  int depth_height = 480;
  int depth_fps = 30;
  int depth_subsample_factor = 1;
  bool depth_median_filter_enabled = false;
  int depth_median_filter_size = 5;
  bool depth_min_max_filter_enabled = false;
  int depth_min_max_filter_size = 3;
  float depth_min_max_filter_threshold = 0.3f;

  // IR config.
  bool infrared_enabled = true;
  int infrared_subsample_factor = 1;
  bool infrared_auto_exposure = true;
  int infrared_gain = 0.5;
  int infrared_exposure = 0;
  bool infrared_emitter_enabled = true;

  // Pointcloud config.
  bool pointcloud_enabled = true;

  // Depth control config.
  static constexpr rs::option kDepthControlOptions[10] = {
      rs::option::r200_depth_control_estimate_median_decrement,
      rs::option::r200_depth_control_estimate_median_increment,
      rs::option::r200_depth_control_median_threshold,
      rs::option::r200_depth_control_score_minimum_threshold,
      rs::option::r200_depth_control_score_maximum_threshold,
      rs::option::r200_depth_control_texture_count_threshold,
      rs::option::r200_depth_control_texture_difference_threshold,
      rs::option::r200_depth_control_second_peak_threshold,
      rs::option::r200_depth_control_neighbor_threshold,
      rs::option::r200_depth_control_lr_threshold};

  // DEFAULT:
  // Default settings on chip. Similiar to the medium
  // setting and best for outdoors.
  static constexpr double kDepthControlDefault[10] = {5., 5.,  192., 1., 512.,
                                                      6., 24., 27.,  7., 24.};
  // OFF:
  // Disable almost all hardware-based outlier removal
  static constexpr double kDepthControlOff[10] = {5., 5., 0., 0., 1023.,
                                                  0., 0., 0., 0., 2047.};

  // LOW:
  // Provide a depthmap with a lower number of outliers
  // removed, which has minimal false negatives.
  static constexpr double kDepthControlLow[10] = {5., 5.,  115., 1., 512.,
                                                  6., 18., 25.,  3., 24.};
  // MEDIUM:
  // Provide a depthmap with a medium number of outliers
  // removed, which has balanced approach.
  static constexpr double kDepthControlMedium[10] = {5., 5.,  185., 5.,  505.,
                                                     6., 35., 45.,  45., 14.};

  // OPTIMIZED:
  // Provide a depthmap with a medium/high number of
  // outliers removed. Derived from an optimization function.
  static constexpr double kDepthControlOptimized[10] = {
      5., 5., 175., 24., 430., 6., 48., 47., 24., 12.};
  // HIGH:
  // Provide a depthmap with a higher number of outliers
  // removed, which has minimal false positives.
  static constexpr double kDepthControlHigh[10] = {5., 5.,  235., 27., 420.,
                                                   8., 80., 70.,  90., 12.};

  const double* depth_control_values = kDepthControlDefault;

  static ZR300Config getFromRosParams(const ros::NodeHandle& private_nh);
};

}  // namespace maplab_realsense

#endif  // MAPLAB_REALSENSE_ZR300_CONFIG_H_
