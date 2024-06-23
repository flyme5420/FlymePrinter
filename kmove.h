#ifndef _KMOVE_H
#define _KMOVE_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <memory>
#include <utility>
#include "toolhead.h"

class ToolHead;

class kMove {
public:
    kMove(ToolHead* toolhead, const std::vector<double>& start_pos, const std::vector<double>& end_pos, double speed);

    void limitSpeed(double speed, double accel);
    void moveError(const std::string& msg = "kMove out of range") const;
    void calcJunction(const kMove& prev_move);
    void setJunction(double start_v2, double cruise_v2, double end_v2);
    std::vector<double> start_pos;
    std::vector<double> end_pos;
// private:
    ToolHead* toolhead;
    double accel;
    double junction_deviation;
    std::vector<void (*)()> timing_callbacks;
    bool is_kinematic_move;
    std::vector<double> axes_d;
    double move_d;
    std::vector<double> axes_r;
    double min_move_t;
    double max_start_v2;
    double max_cruise_v2;
    double delta_v2;
    double max_smoothed_v2;
    double smooth_delta_v2;
    double start_v;
    double cruise_v;
    double end_v;
    double accel_t;
    double cruise_t;
    double decel_t;
};

#endif