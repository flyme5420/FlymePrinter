#ifndef _MCU_STEPPER_H
#define _MCU_STEPPER_H

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "mcu.h"
// #include "chelper.h" // Assuming chelper provides necessary FFI and other functions
extern "C" {
    #include "chelper/trapq.h"
    #include "chelper/itersolve.h"
    #include "chelper/stepcompress.h"
}

#define MIN_BOTH_EDGE_DURATION 0.000000200

class MCU_stepper {
public:
    MCU_stepper(std::string name,
            double rotation_dist, int steps_per_rotation, double step_pulse_duration = 0.0,
            bool units_in_radians = false);
    ~MCU_stepper();
    void generate_steps(double flush_time);
    void _set_mcu_position(int mcu_pos);
    void set_stepper_kinematics(struct stepper_kinematics *sk);
    void setup_itersolve(const std::string& alloc_func, std::vector<void*> params);
    std::vector<double> get_commanded_position() const;
    int get_mcu_position() const;
    void set_position(const std::vector<double>& coord);
    std::string get_name(bool short_name = false) const;
    bool units_in_radians() const;
    std::pair<double, bool> get_pulse_duration() const;
    void _build_config();
    struct trapq * get_trapq() const;
    void set_trapq(struct trapq *tq);

private:
    std::string _name;
    double _rotation_dist;
    int _steps_per_rotation;
    double _step_pulse_duration;
    bool _units_in_radians;
    double _step_dist;
    MCU* _mcu;
    int _oid;
    std::string _step_pin;
    bool _invert_step;
    std::string _dir_pin;
    bool _invert_dir;
    bool _orig_invert_dir;
    bool _step_both_edge;
    bool _req_step_both_edge;
    double _mcu_position_offset;
    void* _reset_cmd_tag;
    void* _get_position_cmd;
    std::vector<std::function<void(int)>> _active_callbacks;
    // std::shared_ptr<void> _stepqueue;
    struct stepper_kinematics *_stepper_kinematics;
    // std::shared_ptr<void> _stepper_kinematics;
    struct trapq *_trapq;
    struct stepcompress *_stepqueue;

    std::function<int(struct stepper_kinematics *, double)> _itersolve_check_active;
    std::function<int(struct stepper_kinematics *, double)> _itersolve_generate_steps;
};

#endif