#ifndef __HUMAN_POSE_ESTIMATOR_H
#define __HUMAN_POSE_ESTIMATOR_H
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include<iostream>
#include<string>

using namespace std;
using namespace cv;
using namespace cv::dnn;


class human_pose_estimator
{
private:
    string protoFile = "pose/coco/pose_deploy_linevec.prototxt";
    string weightsFile = "pose/coco/pose_iter_440000.caffemodel";
    bool vis_enable;

public:

    Net net;
    Mat pose;
    Mat pose_image;
    int img_width;
    int img_height;
    float thresh = 0.01;
    float dt;
    human_pose_estimator();
    ~human_pose_estimator(){};

    void set_vis_enable(bool en){vis_enable = en;};
    void set_image_size(int width, int height);
    void pose_estimate(Mat& image);
    void draw_human_pose(Mat& image, Mat& pose);
};

#endif
