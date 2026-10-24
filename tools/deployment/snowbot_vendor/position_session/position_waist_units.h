#pragma once
#include "position_session_contract.h"

namespace position_session::waist_units {
// Experiment in the existing position coordinate, not output-angle calibration.
// The preparer substitutes explicit ratios only in an opt-in candidate.
// 0x606C feedback and 0x60FF/0x60B1 commands share motor counts/s.
// Unit conversion is independent of optional CSP feedforward gain/mapping.
constexpr double motor_per_position[2] = {1.0, 1.0};
constexpr double feedforward_gain = 0.0;
static_assert(motor_per_position[0] >= 1 && motor_per_position[0] <= 200);
static_assert(motor_per_position[1] >= 1 && motor_per_position[1] <= 200);
static_assert(feedforward_gain >= 0 && feedforward_gain <= .1);
constexpr bool selected(unsigned slot) { return slot == 18 || slot == 19; }
inline double feedback_speed(unsigned slot, double motor_speed) {
    return selected(slot) ? motor_speed / motor_per_position[slot - 18] : motor_speed;
}
constexpr uint16_t velocity_object(unsigned alias) {
    return (alias == 19 || alias == 20) && feedforward_gain > 0 ? 0x60b1 : 0x60ff;
}
inline int32_t command_speed(unsigned slot, double position_speed) {
    const double multiplier = selected(slot)
        ? motor_per_position[slot - 18] * (feedforward_gain > 0 ? feedforward_gain : 1.) : 1.;
    return csp_counts(position_speed * multiplier);
}
inline double written_speed(unsigned slot, double position_speed) {
    return feedback_speed(slot, csp_radians(command_speed(slot, position_speed)));
}
} // namespace position_session::waist_units
