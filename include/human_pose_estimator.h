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

#define COCO

class human_pose_estimator
{
private:
    bool vis_enable;

#ifdef MPI
    const int POSE_PAIRS[14][2] =
        {
            {0, 1}, {1, 2}, {2, 3}, {3, 4}, {1, 5}, {5, 6}, {6, 7}, {1, 14}, {14, 8}, {8, 9}, {9, 10}, {14, 11}, {11, 12}, {12, 13}};

    string protoFile = "/home/libing/data/openpose_model/pose/mpi/pose_deploy_linevec_faster_4_stages.prototxt";
    string weightsFile = "/home/libing/data/openpose_model/pose/mpi/pose_iter_160000.caffemodel";

    int nPoints = 15;
#endif

#ifdef COCO
    const int POSE_PAIRS[17][2] =
        {
            {1, 2}, {1, 5}, {2, 3}, {3, 4}, {5, 6}, {6, 7}, {1, 8}, {8, 9}, {9, 10}, {1, 11}, {11, 12}, {12, 13}, {1, 0}, {0, 14}, {14, 16}, {0, 15}, {15, 17}};

    string protoFile = "/home/libing/data/openpose_model/pose/coco/pose_deploy_linevec.prototxt";
    string weightsFile = "/home/libing/data/openpose_model/pose/coco/pose_iter_440000.caffemodel";

    int nPoints = 18;
#endif

public:

    Net net;
    Mat pose;
    Mat pose_image;
    int blob_img_width;
    int blob_img_height;
    float thresh = 0.01;
    float dt;
    human_pose_estimator();
    ~human_pose_estimator(){};

    void set_vis_enable(bool en){vis_enable = en;};
    void set_blob_size(int width, int height);
    void pose_estimate(Mat& image);
    void draw_human_pose(Mat& image, Mat& pose);
};

#endif
