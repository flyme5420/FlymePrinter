
#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <string>
#include <map>
#include <memory>
#include "corexy.h"

class Toolhead; // Forward declaration


CoreXYKinematics::CoreXYKinematics(std::shared_ptr<Toolhead> toolhead, std::shared_ptr<Config> config) {
    // Setup axis rails
    // for (const char* n : {"x", "y", "z"}) {
    //     auto rail = std::make_shared<Rail>(config->getsection("stepper_" + std::string(1, *n)));
    //     rails.push_back(rail);
    // }
    
    // for (auto& s : rails[1]->get_steppers()) {
    //     rails[0]->get_endstops()[0].first->add_stepper(s);
    // }
    
    // for (auto& s : rails[0]->get_steppers()) {
    //     rails[1]->get_endstops()[0].first->add_stepper(s);
    // }
    
    // rails[0]->setup_itersolve("corexy_stepper_alloc", '+');
    // rails[1]->setup_itersolve("corexy_stepper_alloc", '-');
    // rails[2]->setup_itersolve("cartesian_stepper_alloc", 'z');
    
    // for (auto& s : get_steppers()) {
    //     s->set_trapq(toolhead->get_trapq());
    //     toolhead->register_step_generator(std::bind(&Stepper::generate_steps, s));
    // }
    
    // config->get_printer()->register_event_handler("stepper_enable:motor_off",
    //     std::bind(&CoreXYKinematics::_motor_off, this, std::placeholders::_1));
    
    // // Setup boundary checks
    // auto [max_velocity, max_accel] = toolhead->get_max_velocity();
    // max_z_velocity = config->getfloat("max_z_velocity", max_velocity, 0., max_velocity);
    // max_z_accel = config->getfloat("max_z_accel", max_accel, 0., max_accel);
    
    // limits = {{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}};
    // for (auto& r : rails) {
    //     auto range = r->get_range();
    //     axes_min.push_back(range.first);
    //     axes_max.push_back(range.second);
    // }
}

// std::vector<std::shared_ptr<Stepper>> CoreXYKinematics::get_steppers() {
//     std::vector<std::shared_ptr<Stepper>> steppers;
//     for (auto& rail : rails) {
//         auto rs = rail->get_steppers();
//         steppers.insert(steppers.end(), rs.begin(), rs.end());
//     }
//     return steppers;
// }

std::vector<double> CoreXYKinematics::calc_position(const std::map<std::string, double>& stepper_positions) {
    std::vector<double> pos;
    for (auto& rail : rails) {
        // pos.push_back(stepper_positions.at(rail->get_name()));
    }
    return {0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]};
}

void CoreXYKinematics::set_position(const Coord& newpos, const std::vector<int>& homing_axes) {
    for (size_t i = 0; i < rails.size(); ++i) {
        rails[i]->set_position(newpos);
        if (std::find(homing_axes.begin(), homing_axes.end(), i) != homing_axes.end()) {
            limits[i] = rails[i]->get_range();
        }
    }
}

void CoreXYKinematics::note_z_not_homed() {
    // Helper for Safe Z Home
    limits[2] = {1.0, -1.0};
}

// void CoreXYKinematics::home(HomingState& homing_state) {
//     // Each axis is homed independently and in order
//     for (auto axis : homing_state.get_axes()) {
//         auto rail = rails[axis];
//         auto [position_min, position_max] = rail->get_range();
//         // auto hi = rail->get_homing_info();
//         // std::vector<double> homepos = {0.0, 0.0, 0.0, 0.0};
//         // homepos[axis] = hi.position_endstop;
//         // auto forcepos = homepos;
//         // if (hi.positive_dir) {
//         //     forcepos[axis] -= 1.5 * (hi.position_endstop - position_min);
//         // } else {
//         //     forcepos[axis] += 1.5 * (position_max - hi.position_endstop);
//         // }
//         // homing_state.home_rails({rail}, forcepos, homepos);
//     }
// }

void CoreXYKinematics::_motor_off(double print_time) {
    limits = {{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}};
}

void CoreXYKinematics::_check_endstops(kMove& move) {
    auto end_pos = move.end_pos;
    for (int i = 0; i < 3; ++i) {
        if (move.axes_d[i] && (end_pos[i] < limits[i].first || end_pos[i] > limits[i].second)) {
            if (limits[i].first > limits[i].second) {
                throw std::runtime_error("Must home axis first");
            }
            throw std::runtime_error("Endstop error");
        }
    }
}

void CoreXYKinematics::check_move(kMove &move) {
    auto limits = this->limits;
    // auto [xpos, ypos] = move.end_pos;
    double xpos = move.end_pos[0];
    double ypos = move.end_pos[1];
    if (xpos < limits[0].first || xpos > limits[0].second ||
        ypos < limits[1].first || ypos > limits[1].second) {
        _check_endstops(move);
    }
    if (!move.axes_d[2]) {
        // Normal XY move - use defaults
        return;
    }
    // Move with Z - update velocity and accel for slower Z axis
    // _check_endstops(move);
    double z_ratio = move.move_d / std::abs(move.axes_d[2]);
    move.limitSpeed(max_z_velocity * z_ratio, max_z_accel * z_ratio);
}

// std::map<std::string, double> CoreXYKinematics::get_status(double eventtime) {
//     std::string axes;
//     for (size_t i = 0; i < limits.size(); ++i) {
//         if (limits[i].first <= limits[i].second) {
//             axes += "xyz"[i];
//         }
//     }
//     return {
        // {"homed_axes", axes},
        // {"axis_minimum", axes_min},
        // {"axis_maximum", axes_max}
//     };
// }
    
// std::shared_ptr<CoreXYKinematics> CoreXYKinematics::load_kinematics(std::shared_ptr<Toolhead> toolhead, std::shared_ptr<Config> config) {
//     return std::make_shared<CoreXYKinematics>(toolhead, config);
// }
