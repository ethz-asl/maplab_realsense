#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <maplab-realsense/zr300.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zr300node");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  maplab_realsense::ZR300 cam(nh, pnh);

  const bool success = cam.start();
  if (!success) {
    LOG(ERROR) << "Starting the camera failed.";
    return EXIT_FAILURE;
  }

  LOG(INFO) << "Camera initialized.";

  ros::spin();

  LOG(INFO) << "Stopping the camera.";
  cam.stop();
}
