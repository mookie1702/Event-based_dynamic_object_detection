#include "object_segmentation.h"

void ObjectSegmentation::LoadImg(const cv::Mat &event_count, const cv::Mat &time_img) {
    event_count_ = event_count;
    cv::normalize(time_img, compensated_img_, 0, 255, cv::NORM_MINMAX);
    compensated_img_.convertTo(compensated_img_, CV_8UC1);
}

void ObjectSegmentation::ObjectSegment() {
    ClearData();

    /* Clustering */
    dbscan_.reset(new DBSCAN(k_DBSCAN_Eps_, k_DBSCAN_MinPts_));
    dbscan_->GetDataPointsInImg(compensated_img_);
    dbscan_->GetDistanceMatrix();
    dbscan_->Cluster();
    // dbscan_->DisplayCluster();

    data_set_ = dbscan_->GetDataset();
    cluster_number_ = dbscan_->GetClusterNumber();
    if (1 < cluster_number_) {
        GetObjectNumber();
        cout << "The number of objects is: " << object_size_ << endl;
    }

    /* Optical Flow */
    if (0 < object_size_) {
        is_object_ = true;
        CalcFarnebackOpticalFlow();
        // DisplayObject();
    }
}

void ObjectSegmentation::ClearData() {
    is_object_ = false;
    cluster_number_ = 0;
    object_size_ = 0;
    object_number_.clear();
}

void ObjectSegmentation::GetObjectNumber() {
    for (int i = 1; i < cluster_number_; i++) {
        int point_counter = 0;
        for (auto point : data_set_) {
            if (i == point.cluster_ID_) {
                point_counter++;
            }
        }
        if (k_object_threshold_ < point_counter) {
            object_size_ += 1;
            object_number_.push_back(i);
        } else if (k_object_threshold_ >= point_counter) {
            int k = 0;
            for (auto iter = data_set_.begin(); iter != data_set_.end(); k++) {
                if (i == data_set_[k].cluster_ID_) {
                    iter = data_set_.erase(iter);
                    k -= 1;
                    continue;
                }
                iter++;
            }
        }
    }
}

void ObjectSegmentation::CalcLKOpticalFlow() {
    static bool need_find_keypoint = true;
    static list<cv::Point2f> keypoints;
    static cv::Mat last_img, img;

    img = compensated_img_.clone();

    if (need_find_keypoint) {
        // detect Fast Feature in the first frame.
        vector<cv::KeyPoint> kps;
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
        detector->detect(img, kps);
        for (auto kp : kps)
            keypoints.push_back(kp.pt);
        last_img = img;
        need_find_keypoint = false;
        return;
    }

    // track the features above in the next frames by lk methods.
    vector<cv::Point2f> next_keypoints;
    vector<cv::Point2f> prev_keypoints;
    for (auto kp : keypoints)
        prev_keypoints.push_back(kp);
    vector<unsigned char> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(last_img, img, prev_keypoints, next_keypoints, status, error);

    // delete the missing keypoints.
    int i = 0;
    for (auto iter = keypoints.begin(); iter != keypoints.end(); i++) {
        if (status[i] == 0) {
            iter = keypoints.erase(iter);
            continue;
        }
        *iter = next_keypoints[i];
        iter++;
    }
    cout << "tracked keypoints: " << keypoints.size() << endl;
    if (keypoints.size() < 500) {
        need_find_keypoint = true;
    }

    cv::Mat img_show = img.clone();
    for (auto kp : keypoints)
        cv::circle(img_show, kp, 1, cv::Scalar(255, 255, 255), 1);
    cv::imshow("corners", img_show);
    cv::waitKey(0);

    last_img = img;
}

void ObjectSegmentation::CalcLKOpticalFlow(vector<DataPoint>& dataset) {
    static bool first_running = true;
    static list<cv::Point2f> keypoints;
    static cv::Mat last_img, img;

    img = compensated_img_.clone();

    if (first_running) {
        last_img = img;
        first_running = false;
        return;
    }

    for (auto point : data_set_) {
        if (CORE == point.point_type_) {
            cv::Point2f tmp;
            tmp.x = point.x_;
            tmp.y = point.y_;
            keypoints.push_back(tmp);
        }
    }

    vector<cv::Point2f> next_keypoints;
    vector<cv::Point2f> prev_keypoints;
    for (auto kp : keypoints)
        prev_keypoints.push_back(kp);
    vector<unsigned char> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(last_img, img, prev_keypoints, next_keypoints, status, error);

    // delete the missing keypoints.
    int i = 0;
    for (auto iter = keypoints.begin(); iter != keypoints.end(); i++) {
        if (status[i] == 0) {
            iter = keypoints.erase(iter);
            continue;
        }
        *iter = next_keypoints[i];
        iter++;
    }
    cout << "tracked keypoints: " << keypoints.size() << endl;

    cv::Mat img_show = img.clone();
    for (auto kp : keypoints)
        cv::circle(img_show, kp, 1, cv::Scalar(255, 255, 255), 1);
    cv::imshow("optical_flow", img_show);
    cv::waitKey(0);

    last_img = img;
}

void ObjectSegmentation::CalcFarnebackOpticalFlow() {
    static bool first_runing = true;
    static cv::Mat last_img, img;

    img = compensated_img_.clone();

    if (first_runing) {
        last_img = img;
        first_runing = false;
        return;
    }

    cv::calcOpticalFlowFarneback(last_img, img, flow_data_, 0.5, 3, 15, 3, 5, 1.2, 0);
    last_img = img;

    /* display the effect of optical flow. */
    // cv::Mat img_show;
    // img_show = img.clone();
    // for (int y = 0; y < img_show.rows; y += 16) {
    //     for (int x = 0; x < img_show.cols; x += 16) {
    //         const cv::Point2f &fxy = flow_data_.at<cv::Point2f>(y, x);
    //         line(img_show, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), cv::Scalar(255, 255, 255));
    //         cv::circle(img_show, cv::Point(x, y), 2, cv::Scalar(255, 255, 255), -1);
    //     }
    // }
    // cv::imshow("Farneback_Optical_Flow", img_show);
    // cv::waitKey(0);

    bool is_base = true;
    cv::Point2f base, tmp;
    int tmp_object_size = object_size_;

    for (int i = 0; i < tmp_object_size; i++) {
        for (auto point : data_set_) {
            if (object_number_[i] == point.cluster_ID_ && true == is_base) {
                base = flow_data_.at<cv::Point2f>(point.y_, point.x_);
                is_base = false;
            } else if (object_number_[i] == point.cluster_ID_ && false == is_base) {
                tmp = flow_data_.at<cv::Point2f>(point.y_, point.x_);
            }
            if (-5 > base.dot(tmp)) {
                object_size_ += 1;
                is_base = true;
                break;
            }
        }
        is_base = true;
    }
}

void ObjectSegmentation::DisplayObject() {
    cv::Mat display_img = compensated_img_.clone();
    cv::cvtColor(display_img, display_img, cv::COLOR_GRAY2RGB);

    for (auto point : data_set_) {
        cv::Point2f tmp_point;
        tmp_point.x = point.y_;
        tmp_point.y = point.x_;
        cv::circle(display_img, tmp_point, 1, cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("object", display_img);
    cv::waitKey(0);
}
