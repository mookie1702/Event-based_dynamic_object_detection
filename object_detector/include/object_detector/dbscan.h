#ifndef DBSCAN_H
#define DBSCAN_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

enum PointType {
    Unknown,
    Core,
    Border,
    Noise
};

typedef struct DataPoint_ {
    float x_, y_;
    bool is_visited_;
    int point_type_;
    unsigned int cluster_ID_;
} DataPoint;

class DBSCAN {
private:
    // DBSCAN parameters
    float Eps_;
    unsigned int MinPts_;
    float k_w_distance_;
    float k_w_optical_flow_;
    float k_w_time_score_;

    vector<DataPoint> data_set_;

public:
    typedef std::unique_ptr<DBSCAN> Ptr;

    DBSCAN(float eps, unsigned int minpts) : Eps_(eps), MinPts_(minpts) {}
    ~DBSCAN(){}

    void GetData(cv::Mat time_img);
    float GetDistance(DataPoint &point1, DataPoint &point2);
    
};

#endif