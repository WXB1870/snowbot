#pragma once
#include <cmath>

// Exact step of a critically damped position follower. Targets may be
// discontinuous; position and velocity are retained across target updates.
struct TopicJointState {
    double position = 0.0;
    double velocity = 0.0;
    double target = 0.0;

    void step(double dt) {
        constexpr double omega = 4.0; // rad/s bandwidth, not a motor speed limit
        const double error = position - target;
        const double c = velocity + omega * error;
        const double decay = std::exp(-omega * dt);
        position = target + (error + c * dt) * decay;
        velocity = (velocity - omega * c * dt) * decay;
    }
};
