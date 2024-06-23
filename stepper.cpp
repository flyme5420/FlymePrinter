#include "stepper.h"


MCU_stepper::MCU_stepper(std::string name,
            double rotation_dist, int steps_per_rotation, double step_pulse_duration,
            bool units_in_radians)
{
    // if (dir_pin_params.chip != _mcu) {
    //     throw _mcu->get_printer().config_error("Stepper dir pin must be on same mcu as step pin");
    // }

    // _mcu->register_config_callback([this]() { _build_config(); });

    // auto ffi = chelper::get_ffi();
    _stepqueue = stepcompress_alloc(_oid);   //, ffi.second.stepcompress_free);
    stepcompress_set_invert_sdir(_stepqueue, _invert_dir);
    // _mcu->register_stepqueue(_stepqueue.get());

    _itersolve_generate_steps = itersolve_generate_steps;
    _itersolve_check_active = itersolve_check_active;
}

MCU_stepper::~MCU_stepper()
{
    stepcompress_free(_stepqueue);
}

// MCU* MCU_stepper::get_mcu() const {
//     return _mcu;
// }

std::string MCU_stepper::get_name(bool short_name) const {
    if (short_name && _name.find("stepper_") == 0) {
        return _name.substr(8);
    }
    return _name;
}

bool MCU_stepper::units_in_radians() const {
    return _units_in_radians;
}

std::pair<double, bool> MCU_stepper::get_pulse_duration() const {
    return std::make_pair(_step_pulse_duration, _step_both_edge);
}

void MCU_stepper::generate_steps(double flush_time) {
    if (!_active_callbacks.empty()) {
        int ret = _itersolve_check_active(_stepper_kinematics, flush_time);
        if (ret) {
            auto cbs = _active_callbacks;
            _active_callbacks.clear();
            for (const auto& cb : cbs) {
                cb(ret);
            }
        }
    }

    int ret = _itersolve_generate_steps(_stepper_kinematics, flush_time);
    if (ret) {
        throw std::runtime_error("Internal error in stepcompress");
    }
}

void MCU_stepper::_build_config() {
    if (_step_pulse_duration == 0.0) {
        _step_pulse_duration = 0.000002;
    }
    // bool invert_step = _invert_step;
    // int sbe = std::stoi(_mcu->get_constants().at("STEPPER_BOTH_EDGE"));
    // if (_req_step_both_edge && sbe && _step_pulse_duration <= MIN_BOTH_EDGE_DURATION) {
    //     _step_both_edge = true;
    //     _step_pulse_duration = 0.0;
    //     invert_step = -1;
    // }
    // int step_pulse_ticks = _mcu->seconds_to_clock(_step_pulse_duration);
    // _mcu->add_config_cmd("config_stepper oid=" + std::to_string(_oid) + " step_pin=" + _step_pin + " dir_pin=" + _dir_pin +
    //                         " invert_step=" + std::to_string(invert_step) + " step_pulse_ticks=" + std::to_string(step_pulse_ticks));
    // _mcu->add_config_cmd("reset_step_clock oid=" + std::to_string(_oid) + " clock=0", true);
    // _reset_cmd_tag = _mcu->lookup_command("reset_step_clock oid=%c clock=%u").get_command_tag();
    // _get_position_cmd = _mcu->lookup_query_command("stepper_get_position oid=%c", "stepper_position oid=%c pos=%i", _oid).get_command_tag();
    // int max_error_ticks = _mcu->seconds_to_clock(_mcu->get_max_stepper_error());
    // stepcompress_fill(_stepqueue.get(), max_error_ticks, _reset_cmd_tag, _get_position_cmd);
}

void MCU_stepper::_set_mcu_position(int mcu_pos) {
    double mcu_pos_dist = mcu_pos * _step_dist;
    // _mcu_position_offset = mcu_pos_dist - get_commanded_position()[0];
}

void MCU_stepper::setup_itersolve(const std::string& alloc_func, std::vector<void*> params) {
    // auto ffi = chelper::get_ffi();
    // _stepper_kinematics = std::shared_ptr<void>(ffi.first.lookup_function(alloc_func)(params), ffi.second.free);
    // set_stepper_kinematics(_stepper_kinematics);
}

void MCU_stepper::set_position(const std::vector<double>& coord) {
    int mcu_pos = get_mcu_position();
    itersolve_set_position(_stepper_kinematics, coord[0], coord[1], coord[2]);
    _set_mcu_position(mcu_pos);
}

std::vector<double> MCU_stepper::get_commanded_position() const {
    double position = itersolve_get_commanded_pos(_stepper_kinematics);
    return std::vector<double>{position};
}

int MCU_stepper::get_mcu_position() const {
    double mcu_pos_dist = get_commanded_position()[0] + _mcu_position_offset;
    double mcu_pos = mcu_pos_dist / _step_dist;
    return static_cast<int>(mcu_pos >= 0.0 ? mcu_pos + 0.5 : mcu_pos - 0.5);
}

void MCU_stepper::set_stepper_kinematics(struct stepper_kinematics *sk) {
    int mcu_pos = 0;
    if (_stepper_kinematics) {
        mcu_pos = get_mcu_position();
    }
    _stepper_kinematics = sk;

    itersolve_set_stepcompress(_stepper_kinematics, _stepqueue, _step_dist);
    set_trapq(_trapq);
    _set_mcu_position(mcu_pos);
}

struct trapq * MCU_stepper::get_trapq() const {
    return _trapq;
}

void MCU_stepper::set_trapq(struct trapq *tq) {
    if (!tq) {
        tq = nullptr;
    }
    itersolve_set_trapq(_stepper_kinematics, tq);
    _trapq = tq;
}