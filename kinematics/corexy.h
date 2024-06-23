#ifndef _COREXY_H
#define _COREXY_H

#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <string>
#include <map>
#include <memory>
#include "../kmove.h"
#include "../stepper.h"
#include "../config.h"

class Toolhead; // Forward declaration
class MCU_stepper;
class Config;

class Coord {

};

class Rail {
public:
    // std::vector<std::shared_ptr<Stepper>> get_steppers() {
    //     // Implement this method to return steppers associated with this rail
    // }
    
    // std::vector<std::pair<std::shared_ptr<Endstop>, int>> get_endstops() {
    //     // Implement this method to return endstops associated with this rail
    // }
    
    void setup_itersolve(const std::string& solver_type, char axis) {
        // Implement this method to set up iterative solver
    }
    
    std::pair<double, double> get_range() {
        // Implement this method to return the range of this rail
    }
    
    // HomingInfo get_homing_info() {
    //     // Implement this method to return homing info for this rail
    // }
    
    void set_position(const Coord& newpos) {
        // Implement this method to set position of this rail
    }
};

class CoreXYKinematics {
public:
    CoreXYKinematics(std::shared_ptr<Toolhead> toolhead, std::shared_ptr<Config> config);
    void set_position(const Coord& newpos, const std::vector<int>& homing_axes);
    void check_move(kMove& move);
    // void home(HomingState& homing_state);
    void _motor_off(double print_time);
    void _check_endstops(kMove& move);
    void note_z_not_homed();
    std::vector<double> calc_position(const std::map<std::string, double>& stepper_positions);
    // std::map<std::string, double> get_status(double eventtime);

private:
    std::vector<std::shared_ptr<Rail>> rails;
    std::vector<std::pair<double, double>> limits;
    std::vector<double> axes_min;
    std::vector<double> axes_max;
    double max_z_velocity;
    double max_z_accel;
};

#endif