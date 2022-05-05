#include "motion_compensation.h"

void MotionCompensation::MotionCompensate() {
    CleanTimeImgAndEventCount();
    AccumulateEvents(&source_time_frame_, &source_event_count_);
    Visualization(source_time_frame_, "source_time_frame_");

#ifdef IMU_BASED
    GetAvgAngularVelocity();
    RotationalCompensation(&time_img_, &event_count_);
    MorphologicalOperation(&compensated_time_img_);

    Visualization(compensated_time_img_, "compensated_time_img_");
#endif

#ifdef OPTIMIZATION
    int img_cols = static_cast<int>(IMG_COLS / k_d_) + 1;
    int img_rows = static_cast<int>(IMG_ROWS / k_d_) + 1;
    time_img_ = cv::Mat::zeros(cv::Size(img_cols, img_rows), CV_32FC1);
    event_count_ = cv::Mat::zeros(cv::Size(img_cols, img_rows), CV_8UC1);

    CleanWarpParameter();
    WarpEventCloud(prev_M_G_);
    GetTimestampImg(&time_img_, &event_count_);
    UpdateModel(time_img_);
    while (k_xi_ < sqrt(pow((prev_M_G_.h_x - M_G_.h_x), 2) +
                        pow((prev_M_G_.h_y - M_G_.h_y), 2) +
                        pow((prev_M_G_.h_z - M_G_.h_z), 2) +
                        pow((prev_M_G_.theta - M_G_.theta), 2)))
    {
        WarpEventCloud(M_G_);
        GetTimestampImg(&time_img_, &event_count_);
        prev_M_G_.h_x = M_G_.h_x;
        prev_M_G_.h_y = M_G_.h_y;
        prev_M_G_.h_z = M_G_.h_z;
        prev_M_G_.theta = M_G_.theta;
        UpdateModel(time_img_);
    }

    Visualization(time_img_, "time_img_");
    warped_event_buffer_.clear();
#endif

    IMU_buffer_.clear();
    event_buffer_.clear();
}

void MotionCompensation::CleanTimeImgAndEventCount() {
    source_time_frame_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_32FC1);
    time_img_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_32FC1);
    compensated_time_img_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_32FC1);

    source_event_count_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_8UC1);
    event_count_ = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_8UC1);
}

void MotionCompensation::LoadIMU(const sensor_msgs::ImuConstPtr &imu_msg) {
    IMU_buffer_.push_back(*imu_msg);
}

void MotionCompensation::LoadEvent(const dvs_msgs::EventArray::ConstPtr &event_msg) {
    event_buffer_.assign(event_msg->events.begin(), event_msg->events.end());
    event_size_ = event_buffer_.size();
}

void MotionCompensation::AccumulateEvents(cv::Mat *time_img, cv::Mat *event_count) {
    auto t0 = event_buffer_[0].ts;
    dvs_msgs::Event e;
    float delta_T = 0.0f;
    int e_x = 0, e_y = 0;
    int *c;
    float *q;

    for (int i = 0; i < event_size_; i++) {
        e = event_buffer_[i];
        delta_T = (e.ts - t0).toSec();
        e_x = e.x;
        e_y = e.y;

        if (!IsWithinTheBoundary(e_x, e_y, *time_img)) {
            continue;
        } else {
            c = event_count->ptr<int>(e_y, e_x);
            q = time_img->ptr<float>(e_y, e_x);
            *c += 1;
            *q += (delta_T - *q) / (*c);
        }
    }
}

#ifdef IMU_BASED
void MotionCompensation::GetAvgAngularVelocity() {
    omega_avg_.setZero();
    imu_size_ = IMU_buffer_.size();
    if (imu_size_ <= 0) {
        omega_ = 0.0f;
    } else {
        for (int i = 0; i < imu_size_; i++) {
            omega_avg_[0] += IMU_buffer_[i].angular_velocity.x;
            omega_avg_[1] += IMU_buffer_[i].angular_velocity.y;
            omega_avg_[2] += IMU_buffer_[i].angular_velocity.z;
        }
        omega_avg_ = omega_avg_ / static_cast<float>(imu_size_);
        omega_ = omega_avg_.norm();
    }
}

void MotionCompensation::RotationalCompensation(cv::Mat *time_img, cv::Mat *event_count) {
    Eigen::Vector3f rotation_vector;
    Eigen::Vector3f event_vector;
    Eigen::Matrix3f rot_skew_matrix;
    Eigen::Matrix3f rot_K;

    auto t0 = event_buffer_[0].ts;
    float pre_delta_T = 0.0f;
    float delta_T = 0.0f;

    int discretized_x = 0;
    int discretized_y = 0;
    dvs_msgs::Event e;

    int *c;
    float *q;

    for (int i = 0; i < event_size_; i++) {
        e = event_buffer_[i];
        delta_T = (e.ts - t0).toSec();

        /* prepare rotation matrix */
        if (delta_T - pre_delta_T > 1e-3) {
            pre_delta_T = delta_T;
            rotation_vector = omega_avg_ * delta_T;
            rot_skew_matrix = Vector2SkewMatrix(rotation_vector);
            rotation_matrix_ = rot_skew_matrix.exp();
            rot_K = k_event_camera_K_ * rotation_matrix_.transpose() * k_event_camera_K_inverse_;
        }

        /* prepare event vector */
        event_vector[0] = e.x;
        event_vector[1] = e.y;
        event_vector[2] = 1;
        event_vector = rot_K * event_vector;
        ConvertToHomogeneous(event_vector);

        discretized_x = static_cast<int>(event_vector[0]);
        discretized_y = static_cast<int>(event_vector[1]);

        if (IsWithinTheBoundary(discretized_x, discretized_y, *time_img)) {
            c = event_count->ptr<int>(discretized_y, discretized_x);
            q = time_img->ptr<float>(discretized_y, discretized_x);
            *c += 1;
            *q += (delta_T - *q) / (*c);
        }
    }
}

void MotionCompensation::MorphologicalOperation(cv::Mat *time_img) {
    cv::Mat normalized_time_img = cv::Mat::zeros(cv::Size(IMG_COLS, IMG_ROWS), CV_32FC1);
    cv::Mat threshold_img;
    cv::Mat tmp_img;

    cv::normalize(time_img_, normalized_time_img, 0, 1, cv::NORM_MINMAX);

    float threshold = cv::mean(normalized_time_img, event_count_)[0] +
                      threshold_a_ * omega_ + threshold_b_;
    cv::threshold(normalized_time_img, threshold_img, threshold, 1, cv::THRESH_TOZERO);

    /* Gaussian Blur */
    cv::blur(threshold_img, tmp_img, cv::Size(5, 5));
    cv::normalize(tmp_img, tmp_img, 0, 1, cv::NORM_MINMAX);

    /* Morphological Operation */
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_), cv::Point(-1, -1));
    cv::morphologyEx(tmp_img, tmp_img, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

    /* element-wise square to enhance the img contrast */
    tmp_img = tmp_img.mul(tmp_img);
    cv::normalize(tmp_img, tmp_img, 0, 1, cv::NORM_MINMAX);
    *time_img = tmp_img.clone();
}
#endif


#ifdef OPTIMIZATION
void MotionCompensation::CleanWarpParameter() {
    M_G_.h_x = 0.0f;
    M_G_.h_y = 0.0f;
    M_G_.h_z = 0.0f;
    M_G_.theta = 0.0f;
    prev_M_G_.h_x = 0.0f;
    prev_M_G_.h_y = 0.0f;
    prev_M_G_.h_z = 0.0f;
    prev_M_G_.theta = 0.0f;
}

void MotionCompensation::WarpEventCloud(WarpParameter para) {
    auto t0 = event_buffer_[0].ts;
    dvs_msgs::Event e;
    warped_event e1;
    float e_x = 0.0f, e_y = 0.0f, delta_T = 0.0f;

    for (int i = 0; i < event_size_; i++) {
        e = event_buffer_[i];
        delta_T = (e.ts - t0).toSec();
        e_x = static_cast<float>(e.x);
        e_y = static_cast<float>(e.y);

        e1.x = e_x - delta_T * (para.h_x + (para.h_z + 1) * (e_x * cos(para.theta) - e_y * sin(para.theta)) - e_x);
        e1.y = e_y - delta_T * (para.h_y + (para.h_z + 1) * (e_x * sin(para.theta) + e_y * cos(para.theta)) - e_y);
        e1.ts = delta_T;

        warped_event_buffer_.push_back(e1);
    }
    warped_event_size_ = warped_event_buffer_.size();
}

void MotionCompensation::GetTimestampImg(cv::Mat *time_img, cv::Mat *event_count) {
    WarpedEvent e;
    int discretized_x = 0, discretized_y = 0;
    int *c;
    float *q;

    for (int i = 0; i < warped_event_size_; i++) {
        e = warped_event_buffer_[i];
        discretized_x = static_cast<int>(e.x / k_d_) + 1;
        discretized_y = static_cast<int>(e.y / k_d_) + 1;

        if (!IsWithinTheBoundary(discretized_x, discretized_y, *time_img)) {
            continue;
        } else {
            c = event_count->ptr<int>(discretized_y, discretized_x);
            q = time_img->ptr<float>(discretized_y, discretized_x);
            *c += 1;
            *q += (e.ts - *q) / (*c);
        }
    }
}

void MotionCompensation::UpdateModel(cv::Mat &time_img) {
    float tmp_z = 0.0f, tmp_theta = 0.0f, number_I = 0.0f;
    // float alpha_x = 4.0f, alpha_y = 0.0f, alpha_z = 0.0f, alpha_theta = 0.0f;
    // float alpha_x = 0.0f, alpha_y = 10.0f, alpha_z = 0.0f, alpha_theta = 0.0f;
    // float alpha_x = 0.0f, alpha_y = 0.0f, alpha_z = 0.01f, alpha_theta = 0.0f;
    // float alpha_x = 0.0f, alpha_y = 0.0f, alpha_z = 0.0f, alpha_theta = 0.01f;
    float alpha_x = 0.01f, alpha_y = 0.1f, alpha_z = 0.001f, alpha_theta = 0.001f;
    float d_x = 0.0f, d_y = 0.0f, d_z = 0.0f, d_theta = 0.0f;
    cv::Mat G_x, G_y;
    float global_error = 0.0f;

    for (int i = 0; i < IMG_ROWS; i++)
        for (int j = 0; j < IMG_COLS; j++)
            if (abs(time_img.at<float>(i, j)) >= 1e-6)
                number_I += 1;

    cv::Sobel(time_img, G_x, CV_32F, 1, 0);
    cv::Sobel(time_img, G_y, CV_32F, 0, 1);

    d_x = cv::sum(G_x)[0] / number_I;
    d_y = cv::sum(G_y)[0] / number_I;

    for (int i = 0; i < IMG_ROWS; i++) {
        for (int j = 0; j < IMG_COLS; j++) {
            tmp_z += G_x.at<float>(i, j) * i + G_y.at<float>(i, j) * j;
            tmp_theta += G_x.at<float>(i, j) * j - G_y.at<float>(i, j) * i;
            global_error += pow(G_x.at<float>(i, j), 2) + pow(G_y.at<float>(i, j), 2);
        }
    }
    d_z = tmp_z / number_I;
    d_theta = tmp_theta / number_I;

    M_G_.h_x -= d_x * alpha_x;
    M_G_.h_y -= d_y * alpha_y;
    M_G_.h_z -= d_z * alpha_z;
    M_G_.theta -= d_theta * alpha_theta;

    cout << global_error << endl;
}
#endif

void MotionCompensation::Visualization(const cv::Mat img, const string window_name) {
    cv::Mat tmp_img, display_img;
    cv::normalize(img, tmp_img, 0, 255, cv::NORM_MINMAX);
    tmp_img.convertTo(tmp_img, CV_8UC1);
    cv::applyColorMap(tmp_img, display_img, cv::COLORMAP_JET);
    cv::namedWindow(window_name, CV_WINDOW_NORMAL);
    cv::imshow(window_name, display_img);
    cv::waitKey(0);
}

inline bool MotionCompensation::IsWithinTheBoundary(const int &x, const int &y, cv::Mat &img) {
    return (x >= 0 && x < img.cols && y >= 0 && y < img.rows);
}

Eigen::Matrix3f MotionCompensation::Vector2SkewMatrix(Eigen::Vector3f v) {
    Eigen::Matrix3f skew_matrix;
    skew_matrix << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return skew_matrix;
}

void MotionCompensation::ConvertToHomogeneous(Eigen::Vector3f &v) {
    v[0] = v[0] / v[2];
    v[1] = v[1] / v[2];
    v[2] = 1;
}
