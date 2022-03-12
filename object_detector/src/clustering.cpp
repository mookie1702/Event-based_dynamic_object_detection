#include "clustering.h"

void Clustering::Cluster() {

}

void Clustering::LoadCompensatedImgs(const cv::Mat &eventCount, const cv::Mat &timeImg) {
    event_counts_ = eventCount;
    time_img_ = timeImg;
}



