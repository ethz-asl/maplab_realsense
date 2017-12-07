#include "maplab-realsense/zr300-config.h"

#include <glog/logging.h>
#include <string>

#include <ros/ros.h>

namespace maplab_realsense {

constexpr rs::option ZR300Config::kDepthControlOptions[10];
constexpr double ZR300Config::kDepthControlDefault[10];
constexpr double ZR300Config::kDepthControlOff[10];
constexpr double ZR300Config::kDepthControlLow[10];
constexpr double ZR300Config::kDepthControlMedium[10];
constexpr double ZR300Config::kDepthControlOptimized[10];
constexpr double ZR300Config::kDepthControlHigh[10];

const std::string ZR300Config::kFisheyeTopic = "fisheye";
const std::string ZR300Config::kColorTopic = "color";
const std::string ZR300Config::kImuTopic = "imu";
const std::string ZR300Config::kInfraredTopic = "ir_1";
const std::string ZR300Config::kInfrared2Topic = "ir_2";
const std::string ZR300Config::kDepthTopic = "depth";
const std::string ZR300Config::kPointCloudTopic = "pointcloud";
const std::string ZR300Config::kImageSuffix = "image_raw";

ZR300Config ZR300Config::getFromRosParams(const ros::NodeHandle& private_nh) {
  ZR300Config config;

  private_nh.param("imu/enabled", config.imu_enabled, config.imu_enabled);
  if (config.imu_enabled) {
    private_nh.param(
        "imu/imu_skip_first_n_gyro_measurements",
        config.imu_skip_first_n_gyro_measurements,
        config.imu_skip_first_n_gyro_measurements);
  }

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
        "depth/median_filter_size", config.depth_median_filter_size,
        config.depth_median_filter_size);
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

}  // namespace maplab_realsense
