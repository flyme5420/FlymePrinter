#include <utility>
#include "kmove.h"

class ToolHead;

kMove::kMove(ToolHead* toolhead, const std::vector<double>& start_pos, const std::vector<double>& end_pos, double speed) {
    double velocity = std::min(speed, toolhead->max_velocity);
    axes_d = {end_pos[0] - start_pos[0], end_pos[1] - start_pos[1], end_pos[2] - start_pos[2], end_pos[3] - start_pos[3]};
    move_d = std::sqrt(axes_d[0] * axes_d[0] + axes_d[1] * axes_d[1] + axes_d[2] * axes_d[2]);
    double inv_move_d = 0.0;
    if (move_d < 0.000000001) {
        this->end_pos = {start_pos[0], start_pos[1], start_pos[2], end_pos[3]};
        axes_d[0] = axes_d[1] = axes_d[2] = 0.0;
        move_d = std::abs(axes_d[3]);
        if (move_d > 0.0) {
            inv_move_d = 1.0 / move_d;
        }
        accel = 99999999.9;
        velocity = speed;
        is_kinematic_move = false;
    } else {
        inv_move_d = 1.0 / move_d;
    }
    for (auto& d : axes_d) {
        axes_r.push_back(d * inv_move_d);
    }
    min_move_t = move_d / velocity;
    max_start_v2 = 0.0;
    max_cruise_v2 = velocity * velocity;
    delta_v2 = 2.0 * move_d * accel;
    max_smoothed_v2 = 0.0;
    smooth_delta_v2 = 2.0 * move_d * toolhead->max_accel_to_decel;
}

void kMove::limitSpeed(double speed, double accel) {
    double speed2 = speed * speed;
    if (speed2 < max_cruise_v2) {
        max_cruise_v2 = speed2;
        min_move_t = move_d / speed;
    }
    this->accel = std::min(this->accel, accel);
    delta_v2 = 2.0 * move_d * this->accel;
    smooth_delta_v2 = std::min(smooth_delta_v2, delta_v2);
}

void kMove::moveError(const std::string& msg) const {
    std::cerr << msg << ": " << end_pos[0] << " " << end_pos[1] << " " << end_pos[2] << " [" << end_pos[3] << "]" << std::endl;
    // Implement command error handling specific to your application
}

void kMove::calcJunction(const kMove& prev_move) {
    if (!is_kinematic_move || !prev_move.is_kinematic_move) {
        return;
    }
    double extruder_v2 = 0.0;  //toolhead->getExtruder().calcJunction(prev_move, *this);
    const auto& prev_axes_r = prev_move.axes_r;
    double junction_cos_theta = -(axes_r[0] * prev_axes_r[0] + axes_r[1] * prev_axes_r[1] + axes_r[2] * prev_axes_r[2]);
    if (junction_cos_theta > 0.999999) {
        return;
    }
    junction_cos_theta = std::max(junction_cos_theta, -0.999999);
    double sin_theta_d2 = std::sqrt(0.5 * (1.0 - junction_cos_theta));
    double R_jd = sin_theta_d2 / (1.0 - sin_theta_d2);
    double tan_theta_d2 = sin_theta_d2 / std::sqrt(0.5 * (1.0 + junction_cos_theta));
    double move_centripetal_v2 = 0.5 * move_d * tan_theta_d2 * accel;
    double prev_move_centripetal_v2 = 0.5 * prev_move.move_d * tan_theta_d2 * prev_move.accel;
    max_start_v2 = std::min({R_jd * junction_deviation * accel, R_jd * prev_move.junction_deviation * prev_move.accel, move_centripetal_v2, prev_move_centripetal_v2, extruder_v2, max_cruise_v2, prev_move.max_cruise_v2, prev_move.max_start_v2 + prev_move.delta_v2});
    max_smoothed_v2 = std::min(max_start_v2, prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2);
}

void kMove::setJunction(double start_v2, double cruise_v2, double end_v2) {
    double half_inv_accel = 0.5 / accel;
    double accel_d = (cruise_v2 - start_v2) * half_inv_accel;
    double decel_d = (cruise_v2 - end_v2) * half_inv_accel;
    double cruise_d = move_d - accel_d - decel_d;
    start_v = std::sqrt(start_v2);
    cruise_v = std::sqrt(cruise_v2);
    end_v = std::sqrt(end_v2);
    accel_t = accel_d / ((start_v + cruise_v) * 0.5);
    cruise_t = cruise_d / cruise_v;
    decel_t = decel_d / ((end_v + cruise_v) * 0.5);
}