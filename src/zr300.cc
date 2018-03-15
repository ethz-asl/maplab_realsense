#include "maplab-realsense/zr300.h"

#include <glog/logging.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <librealsense/rs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace maplab_realsense {

ZR300::ZR300(
    ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string& frameId)
    : nh_(nh), private_nh_(private_nh), angular_velocity_index_(0u) {
  config_ = ZR300Config::getFromRosParams(private_nh);

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
    return false;
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
  retrieveCameraCalibrations();
  publishStaticTransforms();

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
      [nh](
          const std::string& name,
          const std::string& suffix) -> image_transport::CameraPublisher {
    ros::NodeHandle _nh(*nh, name);
    image_transport::ImageTransport it(_nh);
    return it.advertiseCamera(suffix, 1);
  };

  if (config_.fisheye_enabled) {
    fisheye_publisher_ =
        advertiseCamera(config_.kFisheyeTopic, config_.kImageSuffix);
  }
  if (config_.color_enabled) {
    color_publisher_ =
        advertiseCamera(config_.kColorTopic, config_.kImageSuffix);
  }
  if (config_.infrared_enabled) {
    infrared_publisher_ =
        advertiseCamera(config_.kInfraredTopic, config_.kImageSuffix);
    infrared_2_publisher_ =
        advertiseCamera(config_.kInfrared2Topic, config_.kImageSuffix);
  }
  if (config_.depth_enabled) {
    depth_publisher_ =
        advertiseCamera(config_.kDepthTopic, config_.kImageSuffix);
  }

  if (config_.imu_enabled) {
    imu_publisher_ = nh->advertise<sensor_msgs::Imu>(config_.kImuTopic, 1);
  }

  if (config_.pointcloud_enabled) {
    pointcloud_publisher_ =
        nh->advertise<sensor_msgs::PointCloud2>(config_.kPointCloudTopic, 1);
  }
}

void ZR300::registerCallbacks() {
  if (config_.fisheye_enabled) {
    zr300_device_->set_frame_callback(
        rs::stream::fisheye,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }
  if (config_.color_enabled || config_.pointcloud_enabled) {
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

  if (config_.depth_enabled || config_.pointcloud_enabled) {
    zr300_device_->set_frame_callback(
        rs::stream::depth,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }
}

void ZR300::configureStaticOptions() {
  // This option is necessary to make sure the data is properly synchronized
  // inside the sensor. Keep it turned on.
  zr300_device_->set_option(rs::option::fisheye_strobe, 1);

  // Configure fisheye.
  zr300_device_->set_option(rs::option::fisheye_gain, config_.fisheye_gain);
  zr300_device_->set_option(
      rs::option::fisheye_exposure, config_.fisheye_exposure_ms);

  // Configure color.
  zr300_device_->set_option(
      rs::option::fisheye_color_auto_exposure,
      config_.fisheye_enable_auto_exposure);
  zr300_device_->set_option(rs::option::fisheye_color_auto_exposure_mode, 2);
  // Flicker rate of ambient light.
  zr300_device_->set_option(rs::option::fisheye_color_auto_exposure_rate, 50);

  // Configure depth.
  zr300_device_->set_options(
      config_.kDepthControlOptions, 10u, config_.depth_control_values);
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

void ZR300::retrieveCameraCalibrations() {
  auto getExtrinsics = [](rs::stream stream, rs::device* zr300_device,
                          rs::extrinsics* extrinsics) {
    CHECK_NOTNULL(zr300_device);
    CHECK_NOTNULL(extrinsics);
    try {
      *extrinsics = zr300_device->get_extrinsics(stream, rs::stream::infrared);
      LOG(INFO) << "Retrieved extrinsics for " << stream
                << "\n translation: " << extrinsics->translation[0] << ", "
                << extrinsics->translation[3] << ", "
                << extrinsics->translation[2]
                << "\n rotation: " << extrinsics->rotation[0] << ", "
                << extrinsics->rotation[1] << ", " << extrinsics->rotation[2]
                << ", " << extrinsics->rotation[3] << ", "
                << extrinsics->rotation[4] << ", " << extrinsics->rotation[5]
                << ", " << extrinsics->rotation[6] << ", "
                << extrinsics->rotation[7] << ", " << extrinsics->rotation[8];
    } catch (rs::error& e) {
      LOG(FATAL) << "Failed to retrieve camera extrinsics for stream type: "
                 << stream << "\nerror: " << e.what();
    }
  };

<<<<<<< Updated upstream
auto getMotionExtrinsics = [](rs::stream stream, rs::device *zr300_device,
                              rs::extrinsics *extrinsics) {
  CHECK_NOTNULL(zr300_device);
  CHECK_NOTNULL(extrinsics);
  try {
    *extrinsics = zr300_device->get_motion_extrinsics_from(stream);
    LOG(INFO) << "Retrieved imu to camera extrinsics for " << stream
              << "\n translation: " << extrinsics->translation[0] << ", "
              << extrinsics->translation[3] << ", "
              << extrinsics->translation[2]
              << "\n rotation: " << extrinsics->rotation[0] << ", "
              << extrinsics->rotation[1] << ", " << extrinsics->rotation[2]
              << ", " << extrinsics->rotation[3] << ", "
              << extrinsics->rotation[4] << ", " << extrinsics->rotation[5]
              << ", " << extrinsics->rotation[6] << ", "
              << extrinsics->rotation[7] << ", " << extrinsics->rotation[8];
  } catch (rs::error &e) {
    LOG(FATAL)
        << "Failed to retrieve imu to camera extrinsics for stream type: "
        << stream << "\nerror: " << e.what();
  }
};



  auto getIntrinsics = [](
      rs::stream stream, rs::device* zr300_device, rs::intrinsics* intrinsics) {
=======
  auto getIntrinsics = [](rs::stream stream, rs::device* zr300_device,
                          rs::intrinsics* intrinsics) {
>>>>>>> Stashed changes
    CHECK_NOTNULL(zr300_device);
    CHECK_NOTNULL(intrinsics);
    try {
      *intrinsics = zr300_device->get_stream_intrinsics(stream);
      LOG(INFO) << "Retrieved intrinsics for " << stream << "\nhfov "
                << intrinsics->hfov() << "\nvfov " << intrinsics->vfov()
                << "\ndistortion " << intrinsics->model() << "\nwidth "
                << intrinsics->width << "\nheight " << intrinsics->height
                << "\nppx " << intrinsics->ppx << "\nppy " << intrinsics->ppy
                << "\nfx " << intrinsics->fx << "\nfy " << intrinsics->fy
                << "\ncoeffs " << intrinsics->coeffs[0] << ", "
                << intrinsics->coeffs[1] << ", " << intrinsics->coeffs[2]
                << ", " << intrinsics->coeffs[3] << ", "
                << intrinsics->coeffs[4];
    } catch (rs::error& e) {
      LOG(FATAL) << "Failed to retrieve camera intrinsics for stream type: "
                 << stream << "\nerror: " << e.what();
    }
  };

  getMotionExtrinsics(rs::stream::fisheye, zr300_device_, &T_imu_fisheye_);

  if (config_.fisheye_enabled) {
    getExtrinsics(rs::stream::fisheye, zr300_device_, &T_infrared_fisheye_);
    rs::intrinsics intrinsics_fisheye;
    getIntrinsics(rs::stream::fisheye, zr300_device_, &intrinsics_fisheye);

    convertCalibrationToCameraInfoMsg(
        intrinsics_fisheye, T_infrared_fisheye_, &fisheye_camera_info_);
  }
  if (config_.depth_enabled || config_.pointcloud_enabled) {
    getExtrinsics(rs::stream::depth, zr300_device_, &T_infrared_depth_);
    getIntrinsics(rs::stream::depth, zr300_device_, &intrinsics_depth_);

    convertCalibrationToCameraInfoMsg(
        intrinsics_depth_, T_infrared_depth_, &depth_camera_info_);
  }
  if (config_.infrared_enabled) {
    getExtrinsics(
        rs::stream::infrared2, zr300_device_, &T_infrared_infrared_2_);
    getExtrinsics(rs::stream::infrared, zr300_device_, &T_infrared_infrared_);

    rs::intrinsics intrinsics_infrared;
    getIntrinsics(rs::stream::infrared, zr300_device_, &intrinsics_infrared);
    rs::intrinsics intrinsics_infrared_2;
    getIntrinsics(rs::stream::infrared2, zr300_device_, &intrinsics_infrared_2);

    convertCalibrationToCameraInfoMsg(
        intrinsics_infrared, T_infrared_infrared_, &infrared_camera_info_);
    convertCalibrationToCameraInfoMsg(
        intrinsics_infrared_2, T_infrared_infrared_2_,
        &infrared_2_camera_info_);
  }

  if (config_.color_enabled || config_.pointcloud_enabled) {
    getExtrinsics(rs::stream::color, zr300_device_, &T_infrared_color_);
    getIntrinsics(rs::stream::color, zr300_device_, &intrinsics_color_);

    invertExtrinsics(T_infrared_color_, &T_color_infrared_);

    convertCalibrationToCameraInfoMsg(
        intrinsics_color_, T_infrared_color_, &color_camera_info_);
  }

  try {
    rs::motion_intrinsics imu_intrinsics =
        zr300_device_->get_motion_intrinsics();

    LOG(INFO) << "IMU INTRINSICS: ";
    LOG(INFO) << "acc noise var: " << imu_intrinsics.acc.noise_variances[0]
              << ", " << imu_intrinsics.acc.noise_variances[1] << ", "
              << imu_intrinsics.acc.noise_variances[2];
    LOG(INFO) << "acc bias var: " << imu_intrinsics.acc.bias_variances[0]
              << ", " << imu_intrinsics.acc.bias_variances[1] << ", "
              << imu_intrinsics.acc.bias_variances[2];
    LOG(INFO) << "gyro noise var: " << imu_intrinsics.gyro.noise_variances[0]
              << ", " << imu_intrinsics.gyro.noise_variances[1] << ", "
              << imu_intrinsics.gyro.noise_variances[2];
    LOG(INFO) << "gyro bias var: " << imu_intrinsics.gyro.bias_variances[0]
              << ", " << imu_intrinsics.gyro.bias_variances[1] << ", "
              << imu_intrinsics.gyro.bias_variances[2];
  } catch (rs::error& e) {
    LOG(FATAL) << "Unable to get IMU intrinsics: " << e.what();
  }
}

geometry_msgs::TransformStamped ZR300::convertExtrinsicsToTf(
    const rs::extrinsics& T_to_from, const ros::Time& stamp,
    const std::string& parent, const std::string& child) {
  geometry_msgs::TransformStamped tf_message;
  tf_message.header.stamp = stamp;
  tf_message.header.frame_id = parent;
  tf_message.child_frame_id = child;

  const float* R = T_to_from.rotation;
  tf2::Matrix3x3 rotation_matrix(
      R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]);

  tf2::Quaternion q;
  rotation_matrix.getRotation(q);
  tf_message.transform.rotation.w = q.getW();
  tf_message.transform.rotation.x = q.getX();
  tf_message.transform.rotation.y = q.getY();
  tf_message.transform.rotation.z = q.getZ();

  tf_message.transform.translation.x = T_to_from.translation[0];
  tf_message.transform.translation.y = T_to_from.translation[1];
  tf_message.transform.translation.z = T_to_from.translation[2];

  return tf_message;
}

void ZR300::publishStaticTransforms() {
  std::vector<geometry_msgs::TransformStamped> extrinsics_transforms;
  ros::Time stamp = ros::Time::now();

  extrinsics_transforms.push_back(convertExtrinsicsToTf(
      T_imu_fisheye_, stamp, config_.kImuTopic, config_.kFisheyeTopic));

  const std::string parent = config_.kInfraredTopic;
  extrinsics_transforms.push_back(convertExtrinsicsToTf(
      T_infrared_color_, stamp, parent, config_.kColorTopic));
  extrinsics_transforms.push_back(convertExtrinsicsToTf(
      T_infrared_depth_, stamp, parent, config_.kDepthTopic));
  extrinsics_transforms.push_back(convertExtrinsicsToTf(
      T_infrared_fisheye_, stamp, parent, config_.kFisheyeTopic));
  extrinsics_transforms.push_back(convertExtrinsicsToTf(
      T_infrared_infrared_2_, stamp, parent, config_.kInfrared2Topic));

  // NOTE: this is obviously identity, since IR is the root frame, therefore we
  // commented this out, but keep if for now, in case we change the root frame.
  // extrinsics_transforms.push_back(convertExtrinsicsToTf(
  //     T_infrared_infrared_, stamp, parent, config_.kInfraredTopic));

  extrinsics_broadcaster_.sendTransform(extrinsics_transforms);
}

// Converts the realsense intrinsics and extrinsics to a camera info message.
// Expects the intrinsics of the camera and the extrinsics as T_ir_x, where x is
// the camera frame of the camera we want to publish the info from.
// The base frame for all transformations is the infrared camera.
void ZR300::convertCalibrationToCameraInfoMsg(
    const rs::intrinsics& intrinsics, const rs::extrinsics& T_ir_x,
    sensor_msgs::CameraInfo* camera_info) {
  CHECK_NOTNULL(camera_info);

  static_assert(
      sizeof(intrinsics.coeffs) == 5 * sizeof(float),
      "The intrinsics do not have the expected size (5)!");

  // Copy intrinsics.
  camera_info->width = intrinsics.width;
  camera_info->height = intrinsics.height;
  switch (intrinsics.model()) {
    case rs::distortion::none:
      camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      break;
    case rs::distortion::modified_brown_conrady:
      camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      break;
    case rs::distortion::inverse_brown_conrady:
      camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      break;
    // TODO(mfehr): this distortion model should correspond to the FOV/Fisheye
    // model, not sure what to set in the camera info.
    case rs::distortion::distortion_ftheta:
      camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      break;
    default:
      LOG(FATAL) << "Unknown distortion model: " << intrinsics.model();
  }

  for (int idx = 0; idx < 5; ++idx) {
    camera_info->D.push_back(intrinsics.coeffs[idx]);
  }

  for (int row_idx = 0; row_idx < 3; ++row_idx) {
    for (int col_idx = 0; col_idx < 3; ++col_idx) {
      camera_info->R[row_idx * 3 + col_idx] =
          T_ir_x.rotation[col_idx * 3 + row_idx];
    }
  }

  camera_info->K.assign(0.0);
  camera_info->K[0] = intrinsics.fx;
  camera_info->K[2] = intrinsics.ppx;
  camera_info->K[4] = intrinsics.fy;
  camera_info->K[5] = intrinsics.ppy;
  camera_info->K[8] = 1.0;

  camera_info->P.assign(0.0);
  camera_info->P[0] = intrinsics.fx;
  camera_info->P[2] = intrinsics.ppx;
  camera_info->P[5] = intrinsics.fy;
  camera_info->P[6] = intrinsics.ppy;
  camera_info->P[10] = 1.0;

  rs::extrinsics T_x_ir;
  invertExtrinsics(T_ir_x, &T_x_ir);
  camera_info->P[3] = T_x_ir.translation[0];
  camera_info->P[7] = T_x_ir.translation[1];
  camera_info->P[11] = T_x_ir.translation[2];
}

void ZR300::depthToPointcloud(
    const cv::Mat& rgb_color_image, const cv::Mat& depth_image,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud) {
  CHECK_NOTNULL(pointcloud)->clear();

  for (size_t y_pixels = 0; y_pixels < depth_image.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < depth_image.cols; ++x_pixels) {
      rs::float2 depth_coord;
      depth_coord.x = x_pixels;
      depth_coord.y = y_pixels;

      const size_t depth_idx = x_pixels + depth_image.cols * y_pixels;

      const float z = reinterpret_cast<uint16_t*>(depth_image.data)[depth_idx] *
                      zr300_device_->get_depth_scale();

      if (z > 0.0) {
        const rs::float3 point = intrinsics_depth_.deproject(depth_coord, z);

        rgb_color rgb_image_element;
        pcl::PointXYZRGB point_out;

        rs::float2 image_coord = intrinsics_color_.project(
            T_color_infrared_.transform(T_infrared_depth_.transform(point)));
        image_coord.x = std::round(image_coord.x);
        image_coord.y = std::round(image_coord.y);
        if ((image_coord.x < rgb_color_image.cols) && (image_coord.x >= 0) &&
            (image_coord.y < rgb_color_image.rows) && (image_coord.y >= 0)) {
          const size_t color_image_idx =
              image_coord.x + rgb_color_image.cols * image_coord.y;

          rgb_image_element = reinterpret_cast<rgb_color*>(
              rgb_color_image.data)[color_image_idx];

          point_out.x = point.x;
          point_out.y = point.y;
          point_out.z = point.z;
          point_out.r = rgb_image_element.r;
          point_out.g = rgb_image_element.g;
          point_out.b = rgb_image_element.b;

          pointcloud->push_back(point_out);
        }
      }
    }
  }
}

void ZR300::improveDepth(cv::Mat* depth_image) {
  CHECK_NOTNULL(depth_image);

  if (config_.depth_median_filter_enabled) {
    CHECK_LE(config_.depth_median_filter_size, 5);
    cv::medianBlur(
        *depth_image, *depth_image, config_.depth_median_filter_size);
  }

  if (config_.depth_min_max_filter_enabled) {
    cv::Mat max_image(depth_image->rows, depth_image->cols, CV_16UC1);
    cv::Mat min_image(depth_image->rows, depth_image->cols, CV_16UC1);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(
                            config_.depth_min_max_filter_size,
                            config_.depth_min_max_filter_size));
    cv::erode(*depth_image, min_image, kernel);
    cv::dilate(*depth_image, max_image, kernel);

    const float depth_scale = zr300_device_->get_depth_scale();
    for (size_t i = 0u; i < depth_image->rows * depth_image->cols; ++i) {
      const float difference =
          (reinterpret_cast<uint16_t*>(max_image.data)[i] -
           reinterpret_cast<uint16_t*>(min_image.data)[i]) *
          depth_scale;

      if (difference > config_.depth_min_max_filter_threshold) {
        reinterpret_cast<uint16_t*>(depth_image->data)[i] = 0u;
      }
    }
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

  switch (stream_type) {
    case rs::stream::fisheye: {
      if (fisheye_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, config_.fisheye_fps * 20)
            << "No subscribers for the fisheye images found!";
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

      // Compose camera info msg.
      sensor_msgs::CameraInfoPtr camera_info_msg =
          boost::make_shared<sensor_msgs::CameraInfo>(fisheye_camera_info_);
      camera_info_msg->header = msg->header;

      // Publish image message.
      fisheye_publisher_.publish(msg, camera_info_msg);
      break;
    }
    case rs::stream::color: {
      if (color_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, config_.color_fps * 20)
            << "No subscribers for the color images found!";
        return;
      }

      // Retrieve and buffer color image.
      CHECK_GT(frame.get_frame_number(), latest_color_frame_number_);
      latest_color_frame_number_ = frame.get_frame_number();
      latest_color_image_ =
          cv::Mat(frame.get_height(), frame.get_width(), CV_8UC3);
      memcpy(
          latest_color_image_.data, frame.get_data(),
          latest_color_image_.total() * latest_color_image_.elemSize());

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

      // Compose camera info msg.
      sensor_msgs::CameraInfoPtr camera_info_msg =
          boost::make_shared<sensor_msgs::CameraInfo>(color_camera_info_);
      camera_info_msg->header = msg->header;

      // Publish image.
      color_publisher_.publish(msg, camera_info_msg);

      if (config_.pointcloud_enabled) {
        publishPointCloudIfDataAvailable();
      }
      break;
    }
    case rs::stream::infrared2:
    // Fall through intended.
    case rs::stream::infrared: {
      if (infrared_2_publisher_.getNumSubscribers() == 0 &&
          infrared_publisher_.getNumSubscribers() == 0) {
        LOG_EVERY_N(WARNING, 300)
            << "No subscribers for the infrared images found!";
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

        // Compose camera info msg.
        sensor_msgs::CameraInfoPtr camera_info_msg =
            boost::make_shared<sensor_msgs::CameraInfo>(infrared_camera_info_);
        camera_info_msg->header = msg->header;

        infrared_publisher_.publish(msg, camera_info_msg);
      } else if (stream_type == rs::stream::infrared2) {
        // Check if timestamp is strictly monotonically increasing.
        CHECK_GT(sensor_timestamp_s, last_infrared_2_frame_timestamp_s_);
        last_infrared_2_frame_timestamp_s_ = sensor_timestamp_s;

        msg->header.frame_id = "infrared_2";

        // Compose camera info msg.
        sensor_msgs::CameraInfoPtr camera_info_msg =
            boost::make_shared<sensor_msgs::CameraInfo>(
                infrared_2_camera_info_);
        camera_info_msg->header = msg->header;

        infrared_2_publisher_.publish(msg, camera_info_msg);
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

      improveDepth(&latest_depth_map_);

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

      // Compose camera info msg.
      sensor_msgs::CameraInfoPtr camera_info_msg =
          boost::make_shared<sensor_msgs::CameraInfo>(depth_camera_info_);
      camera_info_msg->header = msg->header;

      // Publish image.
      depth_publisher_.publish(msg, camera_info_msg);

      if (config_.pointcloud_enabled) {
        publishPointCloudIfDataAvailable();
      }
      break;
    }

    default:
      LOG(FATAL) << "Unknown frame type: " << static_cast<int>(stream_type);
  }
}

void ZR300::publishPointCloudIfDataAvailable() {
  // Check if we have depth and color data.
  if (latest_depth_map_.total() == 0 || latest_color_image_.total() == 0) {
    return;
  }

  // Not publishing point cloud, frame number mismatch!
  if (latest_depth_frame_number_ != latest_color_frame_number_) {
    return;
  }

  // Not publishing point cloud, time stamp mismatch!
  if (std::abs(last_color_frame_timestamp_s_ - last_depth_frame_timestamp_s_) >
      1e-6) {
    return;
  }

  // Not re-publishing same frame number for point cloud!
  if (latest_point_cloud_frame_number_ >= latest_depth_frame_number_) {
    return;
  }
  latest_point_cloud_frame_number_ = latest_depth_frame_number_;

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  depthToPointcloud(latest_color_image_, latest_depth_map_, &pointcloud);

  sensor_msgs::PointCloud2 points_msg;
  pcl::toROSMsg(pointcloud, points_msg);
  points_msg.header.frame_id = config_.kDepthTopic;
  points_msg.header.stamp = device_time_translator_->translate(
      last_depth_frame_timestamp_s_ * kMillisecondsToNanoseconds);
  pointcloud_publisher_.publish(points_msg);
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

      ++angular_velocity_index_;
      if (angular_velocity_index_ <
          static_cast<size_t>(config_.imu_skip_first_n_gyro_measurements)) {
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

        msg->angular_velocity.x = item.angular_velocity[0];
        msg->angular_velocity.y = item.angular_velocity[1];
        msg->angular_velocity.z = item.angular_velocity[2];

        msg->linear_acceleration.x = item.acceleration[0];
        msg->linear_acceleration.y = item.acceleration[1];
        msg->linear_acceleration.z = item.acceleration[2];

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

void ZR300::invertExtrinsics(
    const rs::extrinsics& T_A_B_in, rs::extrinsics* T_B_A) {
  CHECK_NOTNULL(T_B_A);

  const rs::extrinsics T_A_B = T_A_B_in;
  for (int row_idx = 0; row_idx < 3; ++row_idx) {
    for (int col_idx = 0; col_idx < 3; ++col_idx) {
      T_B_A->rotation[row_idx * 3 + col_idx] =
          T_A_B.rotation[col_idx * 3 + row_idx];
    }
  }

  const float* R = T_B_A->rotation;
  const float* t = T_A_B.translation;

  T_B_A->translation[0] = -(R[0] * t[0] + R[3] * t[1] + R[6] * t[2]);
  T_B_A->translation[1] = -(R[1] * t[0] + R[4] * t[1] + R[7] * t[2]);
  T_B_A->translation[2] = -(R[2] * t[0] + R[5] * t[1] + R[8] * t[2]);
};

}  // namespace maplab_realsense
