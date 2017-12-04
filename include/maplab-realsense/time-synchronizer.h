#ifndef MAPLAB_REALSENSE_TIME_SYNCHRONIZER_H_
#define MAPLAB_REALSENSE_TIME_SYNCHRONIZER_H_

#include <map>
#include <mutex>

#include <Eigen/Core>
#include <glog/logging.h>

namespace maplab_realsense {

class FrameTimestampSynchronizer {
 public:
  void registerTimestamp(const int frame_idx, const double timestamp) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    const bool emplace_result =
        index_to_timestamp_map_.emplace(frame_idx, timestamp).second;
    CHECK(emplace_result) << "Frame timestamp emplace failed, a duplicate?";
  }

  bool getTimestampForFrame(const int frame_idx, double* timestamp_s) {
    CHECK_NOTNULL(timestamp_s);

    std::lock_guard<std::mutex> lock(map_mutex_);

    typedef std::map<int, double>::const_iterator MapIterator;
    MapIterator it = index_to_timestamp_map_.find(frame_idx);
    if (it == index_to_timestamp_map_.end()) {
      LOG(WARNING) << "No timestamp event received for frame idx " << frame_idx
                   << ".";
      return false;
    }

    *timestamp_s = it->second;

    // Remove all older items to prevent the map from growing.
    index_to_timestamp_map_.erase(index_to_timestamp_map_.begin(), it);
    // index_to_timestamp_map_.erase(it);

    return true;
  }

 private:
  std::map<int, double> index_to_timestamp_map_;
  std::mutex map_mutex_;
};

class ImuSynchronizer {
 public:
  struct ImuData {
    double timestamp;
    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
  };

  void registerGyroMeasurement(
      const double timestamp, const Eigen::Vector3d& data,
      std::vector<ImuData>* synced_imu) {
    CHECK_NOTNULL(synced_imu)->clear();

    std::lock_guard<std::mutex> lock(data_mutex_);
    CHECK_GT(timestamp, last_gyro_measurement_.first);

    const double delta_gyro = timestamp - last_gyro_measurement_.first;

    // Process all accelerometer measurements that happened in the meantime.
    for (const ImuMeasurement& acc_meas : acc_measurements_since_last_gyro_) {
      if (acc_meas.first < last_gyro_measurement_.first ||
          acc_meas.first > timestamp) {
        // Timestamp out of range, continue.
        continue;
      }

      const double delta_t_interp =
          (acc_meas.first - last_gyro_measurement_.first) / delta_gyro;

      ImuData imu_data;
      imu_data.timestamp = acc_meas.first;
      imu_data.acc = acc_meas.second;

      // Interpolate the gyro.
      imu_data.gyro = last_gyro_measurement_.second * (1 - delta_t_interp) +
                      data * delta_t_interp;
      synced_imu->push_back(imu_data);
    }
    acc_measurements_since_last_gyro_.clear();

    last_gyro_measurement_.first = timestamp;
    last_gyro_measurement_.second = data;
  }

  void registerAccelerometerMeasurement(
      const double timestamp, const Eigen::Vector3d& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    CHECK_GT(timestamp, last_acc_timestamp_);

    acc_measurements_since_last_gyro_.emplace_back(timestamp, data);

    last_acc_timestamp_ = timestamp;
  }

 private:
  double last_gyro_timestamp_;
  double last_acc_timestamp_;

  typedef std::pair<double, Eigen::Vector3d> ImuMeasurement;
  std::vector<ImuMeasurement> acc_measurements_since_last_gyro_;
  ImuMeasurement last_gyro_measurement_;

  std::mutex data_mutex_;
};

}  // namespace maplab_realsense

#endif  // MAPLAB_REALSENSE_TIME_SYNCHRONIZER_H_
