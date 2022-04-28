#include "object_segmentation.h"

void ObjectSegmentation::LoadImg(const cv::Mat &event_count, const cv::Mat &time_img) {
    event_count_ = event_count;
    compensated_time_img_ = time_img;
}

void ObjectSegmentation::ObjectSegment() {
    LKFlow();
}

void ObjectSegmentation::LKFlow() {
    static bool need_find_keypoint = true;
    static list<cv::Point2f> keypoints;
    static cv::Mat last_img, img;

    img = compensated_time_img_.clone();

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
    if (keypoints.size() < 50) {
        need_find_keypoint = true;
    }

    cv::Mat img_show = img.clone();
    for (auto kp : keypoints)
        cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
    cv::imshow("corners", img_show);
    cv::waitKey(0);

    last_img = img;
}