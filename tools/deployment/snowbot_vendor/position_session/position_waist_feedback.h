#pragma once
#include <array>
#include <cstdint>
#include <limits>

namespace position_session {
// Loaded once in PREOP. Only feedback uses this motor/output ratio.
class WaistFeedbackUnits {
public:
    struct Axis {
        int32_t reported=0;
        double ratio=0;
        int wkc=0, bytes=0;
        bool provisional=false;
    };
    template<class Reader> bool load(const int* slaves, Reader read) {
        axes_={};
        for(unsigned i=0;i<2;++i) {
            if(slaves[i]<=0)return false;
            auto& a=axes_[i];a.bytes=sizeof(a.reported);
            a.wkc=read(slaves[i],a.bytes,a.reported);
            if(a.wkc<=0) {
                // Explicitly authorized provisional fit; never presented as readback.
                a.ratio=101.;a.provisional=true;
            } else {
                if(a.bytes!=sizeof(a.reported) || !known_ratio(a.reported))return false;
                a.ratio=a.reported;
            }
        }
        return true;
    }
    const Axis& axis(unsigned i)const{return axes_[i];}
    double speed(unsigned slot,double motor_rad_s)const {
        if(slot!=18 && slot!=19)return motor_rad_s;
        const double ratio=axes_[slot-18].ratio;
        return ratio>0 ? motor_rad_s/ratio : std::numeric_limits<double>::quiet_NaN();
    }
private:
    static bool known_ratio(int32_t r){return r==51 || r==81 || r==101 || r==121 || r==161;}
    std::array<Axis,2> axes_{};
};
}
