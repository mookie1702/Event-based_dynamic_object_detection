#ifndef MOTION_ESTIMATION_H
#define MOTION_ESTIMATION_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>

using namespace std;

class MotionEstimation {
private:
public:
    typedef std::unique_ptr<MotionEstimation> Ptr;

    MotionEstimation() {}
    ~MotionEstimation() {}
};

#endif