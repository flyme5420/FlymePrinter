#include <string>
#include <vector>
#include <stdexcept>
// #include "gcode.h"
// #include "chelper.h"
// #include "stepper.h"
// #include "toolhead.h"

#if 0

class ExtruderStepper {
public:
    ExtruderStepper(Config& config) 
        : printer(config.get_printer()), 
          name(config.get_name().substr(config.get_name().find_last_of(' ') + 1)),
          pressure_advance(0.0), 
          pressure_advance_smooth_time(0.0), 
          config_pa(config.getfloat("pressure_advance", 0.0, 0.0)),
          config_smooth_time(config.getfloat("pressure_advance_smooth_time", 0.040, 0.0, 0.200)),
          stepper(config)
    {
        auto ffi_main = chelper::get_ffi();
        sk_extruder = ffi_main.gc(ffi_lib::extruder_stepper_alloc(), ffi_lib::free);
        stepper.set_stepper_kinematics(sk_extruder);
        motion_queue = nullptr;

        printer.register_event_handler("klippy:connect", [this]() { this->_handle_connect(); });

        auto gcode = printer.lookup_object("gcode");
        if (name == "extruder") {
            gcode->register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", nullptr,
                [this](GCode& gcmd) { this->cmd_default_SET_PRESSURE_ADVANCE(gcmd); },
                cmd_SET_PRESSURE_ADVANCE_help);
        }
        gcode->register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", name,
            [this](GCode& gcmd) { this->cmd_SET_PRESSURE_ADVANCE(gcmd); },
            cmd_SET_PRESSURE_ADVANCE_help);
        gcode->register_mux_command("SET_EXTRUDER_ROTATION_DISTANCE", "EXTRUDER", name,
            [this](GCode& gcmd) { this->cmd_SET_E_ROTATION_DISTANCE(gcmd); },
            cmd_SET_E_ROTATION_DISTANCE_help);
        gcode->register_mux_command("SYNC_EXTRUDER_MOTION", "EXTRUDER", name,
            [this](GCode& gcmd) { this->cmd_SYNC_EXTRUDER_MOTION(gcmd); },
            cmd_SYNC_EXTRUDER_MOTION_help);
    }

    void _handle_connect() {
        auto toolhead = printer.lookup_object("toolhead");
        toolhead->register_step_generator([this](double time) { this->stepper.generate_steps(time); });
        _set_pressure_advance(config_pa, config_smooth_time);
    }

    std::map<std::string, double> get_status(double eventtime) {
        return {
            {"pressure_advance", pressure_advance},
            {"smooth_time", pressure_advance_smooth_time},
            {"motion_queue", motion_queue ? 1.0 : 0.0}
        };
    }

    double find_past_position(double print_time) {
        auto mcu_pos = stepper.get_past_mcu_position(print_time);
        return stepper.mcu_to_commanded_position(mcu_pos);
    }

    void sync_to_extruder(const std::string& extruder_name) {
        auto toolhead = printer.lookup_object("toolhead");
        toolhead->flush_step_generation();
        if (extruder_name.empty()) {
            stepper.set_trapq(nullptr);
            motion_queue = nullptr;
            return;
        }
        auto extruder = dynamic_cast<PrinterExtruder*>(printer.lookup_object(extruder_name));
        if (!extruder) {
            throw std::runtime_error("'" + extruder_name + "' is not a valid extruder.");
        }
        stepper.set_position({extruder->last_position, 0.0, 0.0});
        stepper.set_trapq(extruder->get_trapq());
        motion_queue = &extruder_name;
    }

private:
    void _set_pressure_advance(double pressure_advance, double smooth_time) {
        double old_smooth_time = pressure_advance_smooth_time;
        if (!this->pressure_advance) {
            old_smooth_time = 0.0;
        }
        double new_smooth_time = smooth_time;
        if (!pressure_advance) {
            new_smooth_time = 0.0;
        }
        auto toolhead = printer.lookup_object("toolhead");
        toolhead->note_step_generation_scan_time(new_smooth_time * 0.5, old_smooth_time * 0.5);

        ffi_lib::extruder_set_pressure_advance(sk_extruder, pressure_advance, new_smooth_time);
        this->pressure_advance = pressure_advance;
        this->pressure_advance_smooth_time = smooth_time;
    }

    void cmd_default_SET_PRESSURE_ADVANCE(GCode& gcmd) {
        auto extruder = printer.lookup_object("toolhead")->get_extruder();
        if (extruder->extruder_stepper == nullptr) {
            throw std::runtime_error("Active extruder does not have a stepper");
        }
        auto strapq = extruder->extruder_stepper->stepper.get_trapq();
        if (strapq != extruder->get_trapq()) {
            throw std::runtime_error("Unable to infer active extruder stepper");
        }
        extruder->extruder_stepper->cmd_SET_PRESSURE_ADVANCE(gcmd);
    }

    void cmd_SET_PRESSURE_ADVANCE(GCode& gcmd) {
        double pressure_advance = gcmd.get_float("ADVANCE", pressure_advance, 0.0);
        double smooth_time = gcmd.get_float("SMOOTH_TIME", pressure_advance_smooth_time, 0.0, 0.200);
        _set_pressure_advance(pressure_advance, smooth_time);
        std::string msg = "pressure_advance: " + std::to_string(pressure_advance) + "\n" +
                          "pressure_advance_smooth_time: " + std::to_string(smooth_time);
        printer.set_rollover_info(name, name + ": " + msg);
        gcmd.respond_info(msg, false);
    }

    void cmd_SET_E_ROTATION_DISTANCE(GCode& gcmd) {
        double rotation_dist = gcmd.get_float("DISTANCE", 0.0);
        if (rotation_dist != 0.0) {
            bool invert_dir, orig_invert_dir;
            std::tie(invert_dir, orig_invert_dir) = stepper.get_dir_inverted();
            bool next_invert_dir = orig_invert_dir;
            if (rotation_dist < 0.0) {
                next_invert_dir = !orig_invert_dir;
                rotation_dist = -rotation_dist;
            }
            auto toolhead = printer.lookup_object("toolhead");
            toolhead->flush_step_generation();
            stepper.set_rotation_distance(rotation_dist);
            stepper.set_dir_inverted(next_invert_dir);
        } else {
            auto rotation_dist = stepper.get_rotation_distance();
        }
        bool invert_dir, orig_invert_dir;
        std::tie(invert_dir, orig_invert_dir) = stepper.get_dir_inverted();
        if (invert_dir != orig_invert_dir) {
            rotation_dist = -rotation_dist;
        }
        gcmd.respond_info("Extruder '" + name + "' rotation distance set to " + std::to_string(rotation_dist));
    }

    void cmd_SYNC_EXTRUDER_MOTION(GCode& gcmd) {
        std::string ename = gcmd.get("MOTION_QUEUE");
        sync_to_extruder(ename);
        gcmd.respond_info("Extruder '" + name + "' now syncing with '" + ename + "'");
    }

private:
    Printer& printer;
    std::string name;
    double pressure_advance;
    double pressure_advance_smooth_time;
    double config_pa;
    double config_smooth_time;
    PrinterStepper stepper;
    ffi_lib::extruder_stepper_t* sk_extruder;
    const std::string* motion_queue;

    const char* cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters";
    const char* cmd_SET_E_ROTATION_DISTANCE_help = "Set extruder rotation distance";
    const char* cmd_SYNC_EXTRUDER_MOTION_help = "Set extruder stepper motion queue";
};


class PrinterExtruder {
public:
    PrinterExtruder(Config &config, int extruder_num) {
        printer = config.get_printer();
        name = config.get_name();
        last_position = 0.0;

        // Setup hotend heater
        auto pheaters = printer->load_object<Heaters>(config, "heaters");
        std::string gcode_id = "T" + std::to_string(extruder_num);
        heater = pheaters->setup_heater(config, gcode_id);

        // Setup kinematic checks
        nozzle_diameter = config.get_float("nozzle_diameter", 0.0, std::numeric_limits<double>::max());
        double filament_diameter = config.get_float("filament_diameter", nozzle_diameter, std::numeric_limits<double>::max());
        filament_area = M_PI * std::pow(filament_diameter * 0.5, 2);
        double def_max_cross_section = 4.0 * std::pow(nozzle_diameter, 2);
        double def_max_extrude_ratio = def_max_cross_section / filament_area;
        double max_cross_section = config.get_float("max_extrude_cross_section", def_max_cross_section, 0.0, std::numeric_limits<double>::max());
        max_extrude_ratio = max_cross_section / filament_area;
        std::cout << "Extruder max_extrude_ratio=" << max_extrude_ratio << std::endl;

        auto toolhead = printer->lookup_object<Toolhead>("toolhead");
        auto [max_velocity, max_accel] = toolhead->get_max_velocity();
        max_e_velocity = config.get_float("max_extrude_only_velocity", max_velocity * def_max_extrude_ratio, 0.0, std::numeric_limits<double>::max());
        max_e_accel = config.get_float("max_extrude_only_accel", max_accel * def_max_extrude_ratio, 0.0, std::numeric_limits<double>::max());
        max_e_dist = config.get_float("max_extrude_only_distance", 50.0, 0.0, std::numeric_limits<double>::max());
        instant_corner_v = config.get_float("instantaneous_corner_velocity", 1.0, 0.0, std::numeric_limits<double>::max());

        // Setup extruder trapq (trapezoidal motion queue)
        auto [ffi_main, ffi_lib] = chelper::get_ffi();
        trapq = std::shared_ptr<void>(ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free));
        trapq_append = ffi_lib.trapq_append;
        trapq_finalize_moves = ffi_lib.trapq_finalize_moves;

        // Setup extruder stepper
        extruder_stepper = nullptr;
        if (config.has("step_pin") || config.has("dir_pin") || config.has("rotation_distance")) {
            extruder_stepper = std::make_unique<ExtruderStepper>(config);
            extruder_stepper->stepper->set_trapq(trapq);
        }

        // Register commands
        auto gcode = printer->lookup_object<GCode>("gcode");
        if (name == "extruder") {
            toolhead->set_extruder(this, 0.0);
            gcode->register_command("M104", [this](GCommand &gcmd) { this->cmd_M104(gcmd); });
            gcode->register_command("M109", [this](GCommand &gcmd) { this->cmd_M109(gcmd); });
        }
        gcode->register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER", name,
            [this](GCommand &gcmd) { this->cmd_ACTIVATE_EXTRUDER(gcmd); },
            cmd_ACTIVATE_EXTRUDER_help);
    }

    void update_move_time(double flush_time, double clear_history_time) {
        trapq_finalize_moves(trapq.get(), flush_time, clear_history_time);
    }

    std::map<std::string, double> get_status(double eventtime) {
        auto sts = heater->get_status(eventtime);
        sts["can_extrude"] = heater->can_extrude();
        if (extruder_stepper != nullptr) {
            auto estatus = extruder_stepper->get_status(eventtime);
            sts.insert(estatus.begin(), estatus.end());
        }
        return sts;
    }

    std::string get_name() {
        return name;
    }

    std::shared_ptr<Heater> get_heater() {
        return heater;
    }

    std::shared_ptr<void> get_trapq() {
        return trapq;
    }

    std::map<std::string, double> stats(double eventtime) {
        return heater->stats(eventtime);
    }

    void check_move(Move &move) {
        double axis_r = move.axes_r[3];
        if (!heater->can_extrude()) {
            throw std::runtime_error("Extrude below minimum temp\nSee the 'min_extrude_temp' config option for details");
        }
        if ((move.axes_d[0] == 0 && move.axes_d[1] == 0) || axis_r < 0.0) {
            if (std::abs(move.axes_d[3]) > max_e_dist) {
                throw std::runtime_error("Extrude only move too long (" + std::to_string(move.axes_d[3]) + "mm vs " + std::to_string(max_e_dist) + "mm)\nSee the 'max_extrude_only_distance' config option for details");
            }
            double inv_extrude_r = 1.0 / std::abs(axis_r);
            move.limit_speed(max_e_velocity * inv_extrude_r, max_e_accel * inv_extrude_r);
        } else if (axis_r > max_extrude_ratio) {
            if (move.axes_d[3] <= nozzle_diameter * max_extrude_ratio) {
                return;
            }
            double area = axis_r * filament_area;
            std::cout << "Overextrude: " << axis_r << " vs " << max_extrude_ratio << " (area=" << area << " dist=" << move.move_d << ")" << std::endl;
            throw std::runtime_error("Move exceeds maximum extrusion (" + std::to_string(area) + "mm^2 vs " + std::to_string(max_extrude_ratio * filament_area) + "mm^2)\nSee the 'max_extrude_cross_section' config option for details");
        }
    }

    double calc_junction(Move &prev_move, Move &move) {
        double diff_r = move.axes_r[3] - prev_move.axes_r[3];
        if (diff_r != 0.0) {
            return std::pow(instant_corner_v / std::abs(diff_r), 2);
        }
        return move.max_cruise_v2;
    }

    void move(double print_time, Move &move) {
        double axis_r = move.axes_r[3];
        double accel = move.accel * axis_r;
        double start_v = move.start_v * axis_r;
        double cruise_v = move.cruise_v * axis_r;
        bool can_pressure_advance = false;
        if (axis_r > 0.0 && (move.axes_d[0] != 0 || move.axes_d[1] != 0)) {
            can_pressure_advance = true;
        }
        trapq_append(trapq.get(), print_time, move.accel_t, move.cruise_t, move.decel_t, move.start_pos[3], 0.0, 0.0, 1.0, can_pressure_advance, 0.0, start_v, cruise_v, accel);
        last_position = move.end_pos[3];
    }

    double find_past_position(double print_time) {
        if (extruder_stepper == nullptr) {
            return 0.0;
        }
        return extruder_stepper->find_past_position(print_time);
    }

    void cmd_M104(GCommand &gcmd, bool wait = false) {
        double temp = gcmd.get_float("S", 0.0);
        int index = gcmd.get_int("T", -1, 0, std::numeric_limits<int>::max());
        std::shared_ptr<PrinterExtruder> extruder;
        if (index != -1) {
            std::string section = "extruder";
            if (index != 0) {
                section += std::to_string(index);
            }
            extruder = printer->lookup_object<PrinterExtruder>(section);
            if (extruder == nullptr) {
                if (temp <= 0.0) {
                    return;
                }
                throw std::runtime_error("Extruder not configured");
            }
        } else {
            extruder = printer->lookup_object<Toolhead>("toolhead")->get_extruder();
        }
        auto pheaters = printer->lookup_object<Heaters>("heaters");
        pheaters->set_temperature(extruder->get_heater(), temp, wait);
    }

    void cmd_M109(GCommand &gcmd) {
        cmd_M104(gcmd, true);
    }

    const char *cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder";

    void cmd_ACTIVATE_EXTRUDER(GCommand &gcmd) {
        auto toolhead = printer->lookup_object<Toolhead>("toolhead");
        if (toolhead->get_extruder() == this) {
            gcmd.respond_info("Extruder " + name + " already active");
            return;
        }
        gcmd.respond_info("Activating extruder " + name);
        toolhead->flush_step_generation();
        toolhead->set_extruder(this, last_position);
        printer->send_event("extruder:activate_extruder");
    }

private:
    std::shared_ptr<Printer> printer;
    std::string name;
    double last_position;
    std::shared_ptr<Heater> heater;
    double nozzle_diameter;
    double filament_area;
    double max_extrude_ratio;
    double max_e_velocity;
    double max_e_accel;
    double max_e_dist;
    double instant_corner_v;
    std::shared_ptr<void> trapq;
    std::function<void(void*, double, double, double, double, double, double, double, double, double, double, double, double, double)> trapq_append;
    std::function<void(void*, double, double)> trapq_finalize_moves;
    std::unique_ptr<ExtruderStepper> extruder_stepper;
};


class DummyExtruder {
public:
    explicit DummyExtruder(std::shared_ptr<Printer> printer) : printer(std::move(printer)) {}

    void update_move_time(double flush_time, double clear_history_time) {}

    void check_move(Move& move) {
        throw std::runtime_error("Extrude when no extruder present");
    }

    double find_past_position(double print_time) {
        return 0.0;
    }

    double calc_junction(Move& prev_move, Move& move) {
        return move.max_cruise_v2;
    }

    std::string get_name() {
        return "";
    }

    void get_heater() {
        throw std::runtime_error("Extruder not configured");
    }

    void get_trapq() {
        throw std::runtime_error("Extruder not configured");
    }

private:
    std::shared_ptr<Printer> printer;
};

void add_printer_objects(Config& config) {
    auto printer = config.get_printer();
    for (int i = 0; i < 99; ++i) {
        std::string section = "extruder";
        if (i != 0) {
            section += std::to_string(i);
        }
        if (!config.has_section(section)) {
            break;
        }
        auto pe = std::make_shared<PrinterExtruder>(config.getsection(section), i);
        printer->add_object(section, pe);
    }
}
#endif