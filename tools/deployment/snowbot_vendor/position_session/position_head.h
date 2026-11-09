#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace position_session {
// Finite head goals use the existing native smoothstep, with the VR deadline
// retained at the final receiver. Selecting this port retires legacy goals.
class HeadTarget {
    double stamp_=0., deadline_=0., issued_=0.;
    int64_t received_=0, expires_=0;
public:
    static constexpr double max_speed=.75; // Matching Snowbot head profile (rad/s).
    static int64_t steady() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    bool selected() const { return issued_!=0; }
    bool accept(const std::vector<double>& data,double now,int64_t clock) {
        if(data.size()!=6)return false;
        for(double x:data)if(!std::isfinite(x))return false;
        if(std::abs(data[0])>.791 || std::abs(data[1])>.791 ||
           data[2]<stamp_ || data[2]>now || data[3]<=0 || data[3]>.25 ||
           now>=data[2]+data[3] || data[4]<=issued_ || data[4]<data[2] || data[4]>now ||
           (data[5]!=0. && data[5]!=1.))return false;
        stamp_=data[2];deadline_=stamp_+data[3];issued_=data[4];
        received_=clock;expires_=clock+int64_t((deadline_-now)*1e9);
        return true;
    }
    bool expired(double now,int64_t clock) const {
        return selected() && (now<stamp_ || now>=deadline_ || clock<received_ || clock>=expires_);
    }
};
}
