#include "dbscan.h"

void DBSCAN::GetData(cv::Mat &compensated_img) {
    DataPoint tmp;
    tmp.cluster_ID_ = 0;
    tmp.is_visited_ = false;
    tmp.point_type_ = UNCLASSIFIED;

    for (int i = 0; i < compensated_img.rows; i++) {
        for (int j = 0; j < compensated_img.cols; j++) {
            if (127 > compensated_img.at<u_int8_t>(i,j)) {
                continue;
            } else {
                tmp.x_ = i;
                tmp.y_ = j;
                tmp.rho_ = compensated_img.at<u_int8_t>(i,j);
                data_set_.push_back(tmp);
            }
        }
    }
}

void DBSCAN::GetDistanceMatrix() {
    float dis = 0.0f;

    int mat_size = data_set_.size();
    distance_matrix_ = cv::Mat::zeros(cv::Size(mat_size, mat_size), CV_32FC1);

    for (int i = 0; i < mat_size; i++) {
        for (int j = i + 1; j < mat_size; j++) {
            dis = pow((data_set_[i].x_ - data_set_[j].x_), 2)
                + pow((data_set_[i].y_ - data_set_[j].y_), 2);
            distance_matrix_.at<float>(i, j) = dis;
            distance_matrix_.at<float>(j, i) = dis;
        }
    }
}