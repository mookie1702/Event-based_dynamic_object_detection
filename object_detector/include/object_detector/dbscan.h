#ifndef DBSCAN_H
#define DBSCAN_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

#define IMG_ROWS 480
#define IMG_COLS 640

enum PointType {
    UNCLASSIFIED,
    CORE,
    BORDER,
    NOISE
};

typedef struct DataPoint_ {
    float x_, y_;
    float rho_;
    bool is_visited_;
    int point_type_;
    int cluster_ID_;
} DataPoint;

class DBSCAN {
private:
    const int k_rho_threshold_ = 180;

    // DBSCAN parameters
    float Eps_;
    unsigned int MinPts_;
    // const float k_w_distance_;
    // const float k_w_optical_flow_;
    // const float k_w_time_score_;

    vector<DataPoint> data_set_;
    cv::Mat distance_matrix_;
    int data_size_;
    int cluster_number_;

public:
    typedef std::unique_ptr<DBSCAN> Ptr;

    DBSCAN(float eps, unsigned int minpts) {
        Eps_ = eps;
        MinPts_ = minpts;
    }
    ~DBSCAN() {}

    void GetDataPointsInImg(cv::Mat& compensated_img);
    void GetDistanceMatrix();
    void FindPointsInEps(cv::Mat& dis_mat, vector<int>& point_in_eps);
    void Cluster();
    void DisplayCluster();

    vector<DataPoint> GetDataset() { return data_set_; }
    int GetClusterNumber() { return cluster_number_; }
};

#endif