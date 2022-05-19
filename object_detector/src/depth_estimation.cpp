#include "depth_estimation.h"

void DepthEstimation::EstimateDepth(const sensor_msgs::ImageConstPtr& depth_msg) {
    cv_bridge::CvImageConstPtr depth_msg_ptr;
    depth_msg_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);

    cv::Mat depth_gray_u8(depth_msg->height, depth_msg->width, CV_8UC1);
    depth_gray_ = cv::Mat::zeros(cv::Size(depth_msg->height, depth_msg->width), CV_8UC1);

    cv::rgbd::registerDepth(k_depth_camera_intrinsic_, k_event_camera_intrinsic_,
                            k_distort_coeff_, k_RT_event2depth_, depth_msg_ptr->image,
                            k_event_camera_plane_, depth_gray_, false);

    /* Morphology operations */
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
    morphologyEx(depth_gray_, depth_gray_, CV_MOP_CLOSE, kernel, cv::Point(-1, -1), 1);
    depth_gray_.convertTo(depth_gray_u8, CV_8UC1, 1.0 / 256);

    if (is_object_) {
        cv::Rect r(roi_rect_);
        CropDepthImage(depth_gray_, &r);

        cv::Mat obj_img_u8 = depth_gray_u8(r);
        cv::Mat obj_img = depth_gray_(r);

        float u = r.x;
        float v = r.y;

        int loc = SegmentDepth(obj_img_u8);

        if (object_depth_) {
            cv::Mat mask_range;
            cv::inRange(obj_img_u8, loc - 1, loc + 1, mask_range);

            cv::Scalar mean, std;
            cv::meanStdDev(obj_img, mean, std, mask_range);

            if ((std[0] < 100) && (std[0] > 0)) {
                auto m = cv::moments(mask_range, true);

                float roi_u = m.m10 / m.m00;
                float roi_v = m.m01 / m.m00;
                u += roi_u;
                v += roi_v;
                float u0 = k_event_camera_intrinsic_.at<float>(0, 2);
                float v0 = k_event_camera_intrinsic_.at<float>(1, 2);
                float fx = k_event_camera_intrinsic_.at<float>(0, 0);
                float fy = k_event_camera_intrinsic_.at<float>(1, 1);

                float du = (mask_range.cols - roi_u) > roi_u ? roi_u : mask_range.cols - roi_u;
                float dv = (mask_range.rows - roi_v) > roi_v ? roi_v : mask_range.rows - roi_v;

                depth_point_.header.stamp = depth_msg->header.stamp;
                depth_point_.header.frame_id = "/world";
                depth_point_.point.x = mean[0] * (u - u0) / fx;
                depth_point_.point.y = mean[0] * (v - v0) / fy;
                depth_point_.point.z = mean[0];
            }
        }
        /* visualization */
        cv::Mat vis_depth_(depth_msg->height, depth_msg->width, CV_8UC3);
        cv::applyColorMap(depth_gray_u8, vis_depth_, cv::COLORMAP_JET);
        cv::rectangle(vis_depth_, r, cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
        cv::imshow("depth_img", vis_depth_);
        cv::waitKey(0);
    }
}

void DepthEstimation::CropDepthImage(const cv::Mat src, cv::Rect *dst_rect) {
    dst_rect->x = (dst_rect->x - dst_rect->width / 2) < 0 ? 0 : (dst_rect->x - dst_rect->width / 2);
    dst_rect->y = (dst_rect->y - dst_rect->height / 2) < 0 ? 0 : (dst_rect->y - dst_rect->height / 2);
    dst_rect->height *= 2;
    dst_rect->width *= 2;
    if ((dst_rect->height + dst_rect->y) > src.rows)
        dst_rect->height = src.rows - dst_rect->y;
    if ((dst_rect->width + dst_rect->x) > src.cols)
        dst_rect->width = src.cols - dst_rect->x;
}

int DepthEstimation::SegmentDepth(const cv::Mat &img) {
    cv::MatND hist_info;
    const int hist_size = 128;
    float hist_range[] = {1, 128};
    const float *hist_ranges[] = {hist_range};
    const int chs = 0;

    cv::calcHist(&img, 1, &chs, cv::Mat(), hist_info, 1, &hist_size, &hist_ranges[0]);

    int record;
    object_depth_ = false;
    for (record = 0; record < hist_size; record++) {
        if (hist_info.at<int>(record) > 0.02 * img.rows * img.cols) {
            record++;
            object_depth_ = true;
            break;
        }
    }
    return record;
}
