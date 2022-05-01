#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

#include <iostream>
#include <cmath>
#include <ctime>
#include <vector>
#include <string>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "dbscan.h"

using namespace std;

#define IMG_ROWS 480
#define IMG_COLS 640

class ObjectSegmentation {
private:
    /* flags */
    bool is_object_;

    /* parameters */
    // DBSCAN parameters
    const float DBSCAN_Eps_ = 3.0f;
    const int DBSCAN_MinPts_ = 45;

    /* data */
    cv::Mat event_count_;
    cv::Mat compensated_time_img_;

public:
    typedef std::unique_ptr<ObjectSegmentation> Ptr;

    ObjectSegmentation() {
    }
    ~ObjectSegmentation() {}

    void ObjectSegment();
    void LoadImg(const cv::Mat &event_count, const cv::Mat &time_img);
    void OpticalFlow();
};

#endif