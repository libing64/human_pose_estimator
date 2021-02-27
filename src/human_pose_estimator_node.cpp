#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace Eigen;
using namespace cv;

ros::Subscriber camera_info_sub;



void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    camera_info_sub.shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    Mat img = cv_bridge::toCvCopy(image_msg, string("bgr8"))->image;
    imshow("img", img);
    waitKey(2);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mono_vo");
    ros::NodeHandle n("~");

    camera_info_sub = n.subscribe("/camera_info", 10, camera_info_callback);
    ros::Subscriber image_sub = n.subscribe("/image_raw", 10, image_callback);

    ros::spin();
    return 0;
}