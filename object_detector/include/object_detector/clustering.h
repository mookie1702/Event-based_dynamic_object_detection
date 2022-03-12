#ifndef CLUSTERING_H
#define CLUSTERING_H

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

class Clustering {
private:
    /* flags */
    bool is_object_;

    /* parameters */

    /* images */
    cv::Mat event_counts;
    cv::Mat time_img_;
    cv::Mat gray_img_;
    cv::Mat processed_img_;


public:
    Clustering() {
        gray_img_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_8UC1);
        processed_img_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_8UC1);
    }
    ~Clustering();

    
};

#endif