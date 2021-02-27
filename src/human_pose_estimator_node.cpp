#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "human_pose_estimator.h"

using namespace std;
using namespace cv;

ros::Subscriber camera_info_sub;

human_pose_estimator pose_estimator;

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    camera_info_sub.shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    Mat img = cv_bridge::toCvCopy(image_msg, string("bgr8"))->image;
    pose_estimator.pose_estimate(img);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "human_pose_estimator");
    ros::NodeHandle n("~");

    int blob_img_width = 120;
    int blob_img_height = 120;
    n.getParam("blob_img_width", blob_img_width);
    n.getParam("blob_img_height", blob_img_height);
    pose_estimator.set_blob_size(blob_img_width, blob_img_height);

    camera_info_sub = n.subscribe("/camera_info", 1, camera_info_callback);
    ros::Subscriber image_sub = n.subscribe("/image_raw", 1, image_callback);

    ros::spin();
    return 0;
}