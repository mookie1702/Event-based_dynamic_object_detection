#include "object_segmentation.h"

void ObjectSegmentation::LoadImg(const cv::Mat &event_count, const cv::Mat &time_img) {
    event_count_ = event_count;
    cv::normalize(time_img, compensated_img_, 0, 255, cv::NORM_MINMAX);
    compensated_img_.convertTo(compensated_img_, CV_8UC1);
}

void ObjectSegmentation::ObjectSegment() {
    /* Optical Flow */
    // CalcLKOpticalFlow();
    // CalcFarnebackOpticalFlow();

    /* Clustering */
    dbscan_.reset(new DBSCAN(DBSCAN_Eps_, DBSCAN_MinPts_));
    dbscan_->GetDataPointsInImg(compensated_img_);
    dbscan_->GetDistanceMatrix();
    dbscan_->Cluster();
    dbscan_->Display();
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
}
