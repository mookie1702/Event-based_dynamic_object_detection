#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

#include <iostream>
#include <cmath>
#include <ctime>
#include <vector>
#include <string>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;

#define IMG_ROWS 480
#define IMG_COLS 640

class ObjectSegmentation {
private:
    /* flags */
    bool is_object_;

    /* parameters */
    // DBSCAN parameters
    const float DBSCAN_Eps_ = 3.0;
    const int DBSCAN_MinPts_ = 45;

    /* data */
    cv::Mat event_counts_;
    cv::Mat compensated_time_img_;
    cv::Mat gray_img_;


public:
    typedef std::unique_ptr<ObjectSegmentation> Ptr;

    ObjectSegmentation() {
        gray_img_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_8UC1);
    }
    ~ObjectSegmentation() {}

    void ObjectSegment();
};

#endif