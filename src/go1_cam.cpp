//
// Created by qiayuanl on 11/18/23.
//

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <unitree_camera_sdk/UnitreeCameraSDK.hpp>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "go1_cam");

  int deviceNode = 2;             // default 0 -> /dev/video0
  cv::Size frameSize(1856, 800);  // defalut image size: 1856 X 800
  int fps = 30;

  UnitreeCamera cam(deviceNode);
  if (!cam.isOpened()) {
    exit(EXIT_FAILURE);
  }
  cam.setRawFrameSize(frameSize);
  cam.setRawFrameRate(fps);

  std::vector<std::string> camNames = {"front", "head", "left", "right", "abdomen"};
  std::string names = camNames[cam.getPosNumber() - 1];

  ros::NodeHandle leftNh(names + "/left"), rightNh(names + "/right");
  image_transport::ImageTransport leftIt(leftNh), rightIt(rightNh);
  image_transport::CameraPublisher leftPub = leftIt.advertiseCamera("image_raw", 1);
  image_transport::CameraPublisher rightPub = rightIt.advertiseCamera("image_raw", 1);
  camera_info_manager::CameraInfoManager leftInfo(leftNh, names + "_left");
  camera_info_manager::CameraInfoManager rightInfo(rightNh, names + "_right");
  sensor_msgs::CameraInfo leftInfoMsg, rightInfoMsg;
  leftInfoMsg = leftInfo.getCameraInfo();
  rightInfoMsg = rightInfo.getCameraInfo();

  cv::Mat left, right, frame;
  sensor_msgs::ImagePtr msg;
  std_msgs::Header leftHeader, rightHeader;
  leftHeader.frame_id = names + "_left";
  rightHeader.frame_id = names + "_right";
  leftInfoMsg.header.frame_id = names + "_left";
  rightInfoMsg.header.frame_id = names + "_right";

  cam.startCapture();  // start camera capturing

  while (cam.isOpened()) {
    std::chrono::microseconds t;
    if (!cam.getRawFrame(frame, t)) {  // get camera raw image
      usleep(1000);
      continue;
    }

    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(t).count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(t).count() % 1000000000UL;
    ros::Time stamp(sec, nsec);

    leftHeader.stamp = stamp;
    rightHeader.stamp = stamp;
    leftInfoMsg.header.stamp = stamp;
    rightInfoMsg.header.stamp = stamp;

    frame(cv::Rect(0, 0, frame.size().width / 2, frame.size().height)).copyTo(right);
    frame(cv::Rect(frame.size().width / 2, 0, frame.size().width / 2, frame.size().height)).copyTo(left);

    msg = cv_bridge::CvImage(leftHeader, "bgr8", left).toImageMsg();
    leftPub.publish(*msg, leftInfoMsg);
    msg = cv_bridge::CvImage(rightHeader, "bgr8", right).toImageMsg();
    rightPub.publish(*msg, rightInfoMsg);
  }

  cam.stopCapture();  // stop camera capturing

  return 0;
}
