/* --------------------------------------------------------------------------
   ---------------- Environment Perception Module (EPM) - ARAV --------------
   --------------------------------------------------------------------------
   -------------------- Author : Alberto Ceballos Gonzalez ------------------
   -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr ------
   --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved ------
   -------------------------------------------------------------------------- */

/* Import required libraries */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/publisher.h>
#include <unistd.h>

/* Namespaces */
using namespace std;
using namespace cv;
using namespace cv_bridge;
using namespace pcl;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace ros;
using namespace pcl_ros;
using namespace image_transport;

/* Topic names */
std::string TOPIC_RIGHT = "/data/images/right";
std::string TOPIC_LEFT = "/data/images/left";
std::string TOPIC_CLOUD = "/output/cloud";
std::string TOPIC_STATUS = "/output/status";

/* Callbacks */
cv::Mat img_right_msg;
cv::Mat img_left_msg;
bool received_right = false;
bool received_left = false;
void img_right_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(msg, "bgr8");
  img_right_msg = imagePtr->image;
  received_right = true;
}
void img_left_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(msg, "bgr8");
  img_left_msg = imagePtr->image;
  received_left = true;
}

/* Main function */
int main(int argc, char** argv) {

  /* Path configuration */
  std::string base_path(argv[1]);
  std::string data_path = base_path + "/data/calibration.yml";
  std::string output_path = base_path + "/output/output.jpg";

  /* ROS configuration & init */
  ros::init(argc, argv, "depth");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Rate rate(1);

  /* ROS publishers & Subscribers */
  image_transport::Subscriber sub_img_right =
  it.subscribe(TOPIC_RIGHT, 1, img_right_callback);
	image_transport::Subscriber sub_img_left =
  it.subscribe(TOPIC_LEFT, 1, img_left_callback);
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_cloud;
	pub_cloud.advertise(nh, TOPIC_CLOUD, 1);
  sensor_msgs::PointCloud2 rosCloud;
  ros::Publisher pub_status = nh.advertise<std_msgs::Int8> (TOPIC_STATUS, 1);
  std_msgs::Int8 status_msg;
  status_msg.data = 0;
	pub_status.publish(status_msg);

  /* 1 - Wait until images are loaded */
  while (ros::ok() && (received_right==false || received_left==false)) {
    ros::spinOnce();
    rate.sleep();
  }
  cv::Mat img_left = img_left_msg.clone();
  cv::Mat img_right = img_right_msg.clone();
  std::cout << "[Depth-Vision] Both images have been received" << std::endl;

  /* 2 - Load calibration data */
  cv::Mat CM1, CM2, D1, D2, R1, R2, P1, P2, Q;
  cv::FileStorage file(data_path, cv::FileStorage::READ);
  file ["CM1"] >> CM1;
  file ["CM2"] >> CM2;
  file ["D1"] >> D1;
  file ["D2"] >> D2;
  file ["R1"] >> R1;
  file ["R2"] >> R2;
  file ["P1"] >> P1;
  file ["P2"] >> P2;
  file ["Q"] >> Q;
  file.release();

  /* 3 - Apply rectification map to images */
  cv::Mat mapRightX, mapRightY, mapLeftX, mapLeftY;
  cv::initUndistortRectifyMap(CM1, D1, R1, P1, cv::Size(480, 640), CV_32FC1,
  mapRightX, mapRightY);
  cv::initUndistortRectifyMap(CM2, D2, R2, P2, cv::Size(480, 640), CV_32FC1,
  mapLeftX, mapLeftY);
  cv::remap(img_right, img_right, mapRightX, mapRightY,
  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
  cv::remap(img_left, img_left, mapLeftX, mapLeftY,
  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

  /* Improve general results */
  cv::Mat img_right_up = img_right(cv::Rect(0, 0, 480, 345));
  cv::Mat img_right_down = img_right(cv::Rect(0, 365, 480, 270));
  cv::vconcat(img_right_up, img_right_down, img_right);
  cv::Mat img_left_up = img_left(cv::Rect(0, 0, 480, 345));
  cv::Mat img_left_down = img_left(cv::Rect(0, 365, 480, 270));
  cv::vconcat(img_left_up, img_left_down, img_left);

  /* 4 - Crop images */
  cv::Mat img_right_crop = img_right(cv::Rect(130, 130, 280, 280));
  cv::Mat img_left_crop = img_left(cv::Rect(130, 130, 280, 280));
  cv::cvtColor(img_right, img_right, cv::COLOR_BGR2GRAY, 0);
  cv::cvtColor(img_left, img_left, cv::COLOR_BGR2GRAY, 0);

  /* 5 - Configure the SGBM algorithm */
  cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create();
  left_matcher->setMinDisparity(-64);
  left_matcher->setNumDisparities(192);
  left_matcher->setBlockSize(5);
  left_matcher->setPreFilterCap(4);
  left_matcher->setUniquenessRatio(1);
  left_matcher->setSpeckleWindowSize(150);
  left_matcher->setSpeckleRange(2);
  left_matcher->setDisp12MaxDiff(10);
  left_matcher->setP1(600);
  left_matcher->setP2(2400);
  left_matcher->setMode(cv::StereoSGBM::MODE_HH);
  cv::Ptr<cv::StereoMatcher> right_matcher =
  cv::ximgproc::createRightMatcher(left_matcher);
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter =
  cv::ximgproc::createDisparityWLSFilter(left_matcher);
  wls_filter->setLambda(8000);
  wls_filter->setSigmaColor(4);

  /* 6 - Compute depth map */
  double minValue, maxValue;
  uchar value;
  cv::Mat left_disparity, right_disparity, depth_map, color_map;
  left_matcher->compute(img_left, img_right, left_disparity);
  right_matcher->compute(img_right, img_left, right_disparity);
  wls_filter->filter(left_disparity, img_left, depth_map, right_disparity);
  cv::Mat depth_map_crop = depth_map(cv::Rect(130, 130, 280, 280));
  depth_map.convertTo(depth_map, CV_32F, 1./16);
  depth_map_crop.convertTo(depth_map_crop, CV_8U);
  cv::minMaxLoc (depth_map_crop, &minValue, &maxValue);
  float factor = 255.0/(maxValue - minValue);
  float gap = factor*maxValue - 255.0;
  for (int i=0; i<depth_map_crop.rows; i++) {
    for (int j=0; j<depth_map_crop.cols; j++) {
      value = depth_map_crop.at<uchar>(i,j);
      value = value*factor-gap;
      depth_map_crop.at<uchar>(i,j) = value;
    }
  }
  cv::applyColorMap(depth_map_crop, color_map, cv::COLORMAP_MAGMA);

  /* 7 - Perform object detection */
  int boundingBox [4] = {40, 195, 120, 185};
  cv::Point pt1(boundingBox[0], boundingBox[2]);
  cv::Point pt2(boundingBox[1], boundingBox[3]);
  cv::Mat img_detection = img_left_crop.clone();
  cv::rectangle(img_detection, pt1, pt2, cv::Scalar(0,255,0), 2);

  /* 8 - Filter depth map */
  int xMean = static_cast <int> ((boundingBox[0]+boundingBox[1])/2);
  int yMean = static_cast <int> ((boundingBox[2]+boundingBox[3])/2);
  uchar mean_depth_img = depth_map_crop.at<uchar>(yMean, xMean);
  float mean_depth_pcl = depth_map.at<float>(yMean+130, xMean+130);
  std::vector<std::vector<int>> point_list;
  for (int i=0; i<depth_map_crop.rows; i++) {
    for (int j=0; j<depth_map_crop.cols; j++) {
      if (i>boundingBox[2] && i<boundingBox[3] && j>boundingBox[0] && j<boundingBox[1]) {
        depth_map_crop.at<uchar>(i,j) = mean_depth_img;
        depth_map.at<float>(i+130,j+130) = mean_depth_pcl;
        if (i%5==0 && j%5==0) {
          std::vector<int> point = {i+130,j+130};
          point_list.push_back(point);
        }
      }
      else {
        depth_map_crop.at<uchar>(i,j) = 25;
      }
    }
  }
  cv::applyColorMap(depth_map_crop, depth_map_crop, cv::COLORMAP_MAGMA);

  /* 9 - Create point cloud */
  cv::Mat cvCloud;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<int> point (2);
  cv::Point3f cvPoint;
  cloud.width = point_list.size();
  cloud.height = 1;
  cloud.header.frame_id = "map";
  cv::reprojectImageTo3D(depth_map, cvCloud, Q, false, -1);
  while (point_list.size() > 0) {
    point = point_list[point_list.size()-1];
    cvPoint = cvCloud.at<cv::Point3f>(point[0], point[1]);
    pcl::PointXYZ pclPoint;
    pclPoint.x = -cvPoint.z;
    pclPoint.y = cvPoint.x - 0.1;
    pclPoint.z = cvPoint.y;
    cloud.points.push_back(pclPoint);
    point_list.pop_back();
  }
  pcl::toROSMsg(cloud, rosCloud);
  std::cout << "[Depth-Vision] The point cloud has been generated" << std::endl;

  /* 10 - Publish data */
  int nbMsg = 0;
  while (ros::ok () && nbMsg < 10) {
    status_msg.data = 1;
    pub_status.publish(status_msg);
    pub_cloud.publish(rosCloud);
    nbMsg++;
    ros::spinOnce();
		rate.sleep();
  }

  /* 11 - Results */
  cv::Mat output_img;
  cv::hconcat(img_left_crop, color_map, img_left_crop);
  cv::hconcat(img_detection, depth_map_crop, img_detection);
  cv::vconcat(img_left_crop, img_detection, output_img);
  cv::imwrite (output_path, output_img);

}

/* --------------------------------------------------------------------------
   ---------------- Environment Perception Module (EPM) - ARAV --------------
   --------------------------------------------------------------------------
   -------------------- Author : Alberto Ceballos Gonzalez ------------------
   -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr ------
   --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved ------
   -------------------------------------------------------------------------- */
