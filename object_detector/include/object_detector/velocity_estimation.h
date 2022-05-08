#ifndef VELOCITY_ESTIMATION_H
#define VELOCITY_ESTIMATION_H

#include <iostream>
#include <memory>

using namespace std;

class VelocityEstimation {
private:
public:
    typedef std::unique_ptr<VelocityEstimation> Ptr;
    
    VelocityEstimation() {}
    ~VelocityEstimation() {}
};

#endif