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
    /* parameters */
    // DBSCAN parameters
    const float k_DBSCAN_Eps_ = 10.0f;
    const unsigned int k_DBSCAN_MinPts_ = 10;

    const int k_object_threshold_ = 70;

    /* data */
    cv::Mat event_count_;
    cv::Mat compensated_img_;
    cv::Mat flow_data_;
    vector<DataPoint> data_set_;

    int cluster_number_;
    int object_size_;
    vector<int> object_number_;

    DBSCAN::Ptr dbscan_;

public:
    typedef std::unique_ptr<ObjectSegmentation> Ptr;

    ObjectSegmentation() {}
    ~ObjectSegmentation() {}

    void LoadImg(const cv::Mat &event_count, const cv::Mat &time_img);

    void ObjectSegment();
    void ClearData();
    void GetObjectNumber();

    void CalcLKOpticalFlow();
    void CalcLKOpticalFlow(vector<DataPoint>& dataset);
    void CalcFarnebackOpticalFlow();
    void IsSuperposition();
};

#endif