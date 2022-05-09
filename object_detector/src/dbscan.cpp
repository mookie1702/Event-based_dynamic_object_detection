#include "dbscan.h"

void DBSCAN::GetDataPointsInImg(cv::Mat& compensated_img) {
    DataPoint tmp;
    tmp.cluster_ID_ = 0;
    tmp.is_visited_ = false;
    tmp.point_type_ = UNCLASSIFIED;

    for (int i = 0; i < compensated_img.rows; i++) {
        for (int j = 0; j < compensated_img.cols; j++) {
            if (k_rho_threshold_ > compensated_img.at<u_int8_t>(i,j)) {
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
    data_size_ = data_set_.size();
    distance_matrix_ = cv::Mat::zeros(cv::Size(data_size_, data_size_), CV_32FC1);

    for (int i = 0; i < data_size_; i++) {
        for (int j = i + 1; j < data_size_; j++) {
            dis = pow((data_set_[i].x_ - data_set_[j].x_), 2)
                + pow((data_set_[i].y_ - data_set_[j].y_), 2);
            distance_matrix_.at<float>(i, j) = dis;
            distance_matrix_.at<float>(j, i) = dis;
        }
    }
}

void DBSCAN::FindPointsInEps(cv::Mat& dis_mat, vector<int>& point_in_eps) {
    for (int i = 0; i < data_size_; i++) {
        if (dis_mat.at<float>(0, i) < Eps_) {
            point_in_eps.push_back(i);
        }
    }
}

void DBSCAN::Cluster() {
    vector<int> point_in_eps;
    vector<int> tmp_point_in_eps;
    cluster_number_ = 1;
    cv::Mat distance_roi;

    for (int i = 0; i < data_size_; i++) {
        point_in_eps.clear();
        tmp_point_in_eps.clear();

        if (false == data_set_[i].is_visited_) {
            cv::Rect rect(0, i, data_size_, 1);
            distance_roi = distance_matrix_(rect);
            FindPointsInEps(distance_roi, point_in_eps);

            if (1 == point_in_eps.size()) {
                data_set_[i].point_type_ = NOISE;
                data_set_[i].cluster_ID_ = -1;
                data_set_[i].is_visited_ = true;
            } else if (1 < point_in_eps.size() && point_in_eps.size() <= MinPts_) {
                data_set_[i].point_type_ = BORDER;
                data_set_[i].cluster_ID_ = 0;
            } else if (point_in_eps.size() > MinPts_) {
                data_set_[i].point_type_ = CORE;
                for (auto point : point_in_eps)
                    data_set_[point].cluster_ID_ = cluster_number_;
                while (point_in_eps.size() > 0) {
                    data_set_[point_in_eps[0]].is_visited_ = true;

                    cv::Rect rect(0, point_in_eps[0], data_size_, 1);
                    distance_roi = distance_matrix_(rect);
                    FindPointsInEps(distance_roi, tmp_point_in_eps);

                    if (tmp_point_in_eps.size() > 1) {
                        for (auto point1 : tmp_point_in_eps) {
                            data_set_[point1].cluster_ID_ = cluster_number_;
                        }
                        if (tmp_point_in_eps.size() > MinPts_) {
                            data_set_[point_in_eps[0]].point_type_ = CORE;
                        } else {
                            data_set_[point_in_eps[0]].point_type_ = BORDER;
                        }

                        for (int j = 0; j < tmp_point_in_eps.size(); j++) {
                            if (false == data_set_[tmp_point_in_eps[j]].is_visited_) {
                                data_set_[tmp_point_in_eps[j]].is_visited_ = true;
                                point_in_eps.push_back(tmp_point_in_eps[j]);
                                data_set_[tmp_point_in_eps[j]].cluster_ID_ = cluster_number_;
                            }
                        }
                    }
                    tmp_point_in_eps.clear();
                    point_in_eps.erase(point_in_eps.begin());
                }
                cluster_number_ += 1;
            }
        }
    }
    for (int k = 0; k < data_size_; k++) {
        if (0 == data_set_[k].cluster_ID_) {
            data_set_[k].point_type_ = NOISE;
            data_set_[k].cluster_ID_ = -1;
        }
    }
    cout << "The number of clusters is: " << cluster_number_ << endl;
}

void DBSCAN::DisplayCluster() {
    cv::Mat img_show = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
    img_show.convertTo(img_show, CV_8UC3);
    cv::Point kp;
    for (auto point : data_set_) {
        kp.x = point.y_;
        kp.y = point.x_;
        if (NOISE == point.point_type_) {
            cv::circle(img_show, kp, 1, cv::Scalar(255, 0, 0), 1);
        } else if (BORDER == point.point_type_) {
            cv::circle(img_show, kp, 1, cv::Scalar(0, 255, 0), 1);
        } else if (CORE == point.point_type_) {
            cv::circle(img_show, kp, 1, cv::Scalar(0, 0, 255), 1);
        }
    }
    cv::imshow("dbscan", img_show);
    cv::waitKey(0);
}
