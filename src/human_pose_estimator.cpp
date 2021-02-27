#include "human_pose_estimator.h"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace cv::dnn;


human_pose_estimator::human_pose_estimator()
{
    cout << "protoFile: " << protoFile << endl;
    cout << "wightsFile: " << weightsFile << endl;
    net = readNetFromCaffe(protoFile, weightsFile);
    blob_img_width = 150;
    blob_img_height = 150;
}

void human_pose_estimator::set_blob_size(int width, int height)
{
    blob_img_width = width;
    blob_img_height = height;
    cout << "blob img size: " << blob_img_width << "  " << blob_img_height << endl;
}

void human_pose_estimator::pose_estimate(Mat &image)
{
    double t = (double)cv::getTickCount();

    pose_image = image.clone();
    Mat inpBlob = blobFromImage(image, 1.0 / 255, Size(blob_img_width, blob_img_height), Scalar(0, 0, 0), false, false);

    net.setInput(inpBlob);
    pose = net.forward();

    dt = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    draw_human_pose(pose_image, pose);

    imshow("pose_skeleton", image);
    waitKey(1);
}

void human_pose_estimator::draw_human_pose(Mat &image, Mat &pose)
{
    int H = pose.size[2];
    int W = pose.size[3];
    int frame_width = image.cols;
    int frame_height = image.rows;
    // find the position of the body parts
    vector<Point> points(nPoints);
    for (int n = 0; n < nPoints; n++)
    {
        // Probability map of corresponding body's part.
        Mat probMap(H, W, CV_32F, pose.ptr(0, n));

        Point2f p(-1, -1);
        Point maxLoc;
        double prob;
        minMaxLoc(probMap, 0, &prob, 0, &maxLoc);
        if (prob > thresh)
        {
            p = maxLoc;
            p.x *= (float)frame_width / W;
            p.y *= (float)frame_height / H;

            circle(pose_image, cv::Point((int)p.x, (int)p.y), 8, Scalar(0, 255, 255), -1);
            cv::putText(pose_image, cv::format("%d", n), cv::Point((int)p.x, (int)p.y), cv::FONT_HERSHEY_COMPLEX, 1.1, cv::Scalar(0, 0, 255), 2);
        }
        points[n] = p;
    }

    int nPairs = sizeof(POSE_PAIRS) / sizeof(POSE_PAIRS[0]);

    for (int n = 0; n < nPairs; n++)
    {
        // lookup 2 connected body/hand parts
        Point2f partA = points[POSE_PAIRS[n][0]];
        Point2f partB = points[POSE_PAIRS[n][1]];

        if (partA.x <= 0 || partA.y <= 0 || partB.x <= 0 || partB.y <= 0)
            continue;

        line(image, partA, partB, Scalar(0, 255, 255), 8);
        circle(image, partA, 8, Scalar(0, 0, 255), -1);
        circle(image, partB, 8, Scalar(0, 0, 255), -1);
    }
    cv::putText(image, cv::format("time taken = %.2f sec", dt), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, .8, cv::Scalar(255, 50, 0), 2);
}