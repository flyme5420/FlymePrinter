#include "printer_rail.h"

#if 0

PrinterRail::PrinterRail(const Config &config, bool need_position_minmax, float default_position_endstop, bool units_in_radians)
    : stepper_units_in_radians(units_in_radians) {
    add_extra_stepper(config);
    auto mcu_stepper = steppers[0];
    get_name = std::bind(&PrinterStepper::get_name, mcu_stepper, false);

    auto mcu_endstop = endstops[0].first;
    if (mcu_endstop->get_position_endstop) {
        position_endstop = mcu_endstop->get_position_endstop();
    } else if (default_position_endstop == 0.0) {
        position_endstop = config.getfloat("position_endstop");
    } else {
        position_endstop = config.getfloat("position_endstop", default_position_endstop);
    }

    if (need_position_minmax) {
        position_min = config.getfloat("position_min", 0.0);
        position_max = config.getfloat("position_max", position_min);
    } else {
        position_min = 0.0;
        position_max = position_endstop;
    }

    if (position_endstop < position_min || position_endstop > position_max) {
        throw std::runtime_error("position_endstop must be between position_min and position_max");
    }

    homing_speed = config.getfloat("homing_speed", 5.0);
    second_homing_speed = config.getfloat("second_homing_speed", homing_speed / 2.0);
    homing_retract_speed = config.getfloat("homing_retract_speed", homing_speed);
    homing_retract_dist = config.getfloat("homing_retract_dist", 5.0);

    bool homing_positive_dir_config = config.getboolean("homing_positive_dir", false);
    if (homing_positive_dir_config) {
        homing_positive_dir = homing_positive_dir_config;
    } else {
        float axis_len = position_max - position_min;
        if (position_endstop <= position_min + axis_len / 4.0) {
            homing_positive_dir = false;
        } else if (position_endstop >= position_max - axis_len / 4.0) {
            homing_positive_dir = true;
        } else {
            throw std::runtime_error("Unable to infer homing_positive_dir");
        }
    }
}

std::tuple<float, float> PrinterRail::get_range() const {
    return std::make_tuple(position_min, position_max);
}

std::tuple<float, float, float, float, bool, float> PrinterRail::get_homing_info() const {
    return std::make_tuple(homing_speed, position_endstop, homing_retract_speed, homing_retract_dist, homing_positive_dir, second_homing_speed);
}

std::vector<std::shared_ptr<PrinterStepper>> PrinterRail::get_steppers() const {
    return steppers;
}

std::vector<std::pair<std::shared_ptr<PrinterEndstop>, std::string>> PrinterRail::get_endstops() const {
    return endstops;
}

// void PrinterRail::add_extra_stepper(const Config &config) {
//     auto stepper = std::make_shared<PrinterStepper>(config, stepper_units_in_radians);
//     steppers.push_back(stepper);

//     if (!endstops.empty() && config.get("endstop_pin", "").empty()) {
//         endstops[0].first->add_stepper(stepper);
//         return;
//     }

//     std::string endstop_pin = config.get("endstop_pin");
//     auto printer = config.get_printer();
//     auto ppins = printer->lookup_object("pins");
//     auto pin_params = ppins.parse_pin(endstop_pin, true, true);

//     std::string pin_name = pin_params["chip_name"] + ":" + pin_params["pin"];
//     auto endstop_it = endstop_map.find(pin_name);
//     if (endstop_it == endstop_map.end()) {
//         auto mcu_endstop = ppins.setup_pin("endstop", endstop_pin);
//         endstop_map[pin_name] = { mcu_endstop, pin_params["invert"], pin_params["pullup"] };
//         std::string name = stepper->get_name(true);
//         endstops.push_back({ mcu_endstop, name });
//         auto query_endstops = printer->load_object(config, "query_endstops");
//         query_endstops.register_endstop(mcu_endstop, name);
//     } else {
//         auto mcu_endstop = endstop_it->second.endstop;
//         bool changed_invert = pin_params["invert"] != endstop_it->second.invert;
//         bool changed_pullup = pin_params["pullup"] != endstop_it->second.pullup;
//         if (changed_invert || changed_pullup) {
//             throw std::runtime_error("Shared endstop pin must specify the same pullup/invert settings");
//         }
//     }

//     endstop_it->second.endstop->add_stepper(stepper);
// }

void PrinterRail::setup_itersolve(std::function<void()> alloc_func, ...) {
    for (auto &stepper : steppers) {
        stepper->setup_itersolve(alloc_func);
    }
}

void PrinterRail::generate_steps(float flush_time) {
    for (auto &stepper : steppers) {
        stepper->generate_steps(flush_time);
    }
}

void PrinterRail::set_trapq(int trapq) {
    for (auto &stepper : steppers) {
        stepper->set_trapq(trapq);
    }
}

void PrinterRail::set_position(float coord) {
    for (auto &stepper : steppers) {
        stepper->set_position(coord);
    }
}

PrinterRail LookupMultiRail(const Config &config, bool need_position_minmax, float default_position_endstop, bool units_in_radians) {
    PrinterRail rail(config, need_position_minmax, default_position_endstop, units_in_radians);
    // for (int i = 1; i < 99; ++i) {
    //     if (!config.has_section(config.get_name() + std::to_string(i))) {
    //         break;
    //     }
    //     rail.add_extra_stepper(config.getsection(config.get_name() + std::to_string(i)));
    // }
    return rail;
}
#endif