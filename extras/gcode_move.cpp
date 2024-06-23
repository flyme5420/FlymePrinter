#include "gcode_move.h"

GCodeMove::GCodeMove(std::shared_ptr<ToolHead> toolhead) 
    : absolute_coord(true), absolute_extrude(true), speed(25.0), speed_factor(1.0 / 60.0), extrude_factor(1.0)
{
    base_position = {0.0, 0.0, 0.0, 0.0};
    last_position = {0.0, 0.0, 0.0, 0.0};
    homing_position = {0.0, 0.0, 0.0, 0.0};
    move_with_transform = [this](std::vector<double> pos, double spd) { this->toolhead->kmove(pos, spd); };
    position_with_transform = [this]() { return this->toolhead->get_position(); };
}

// void GCodeMove::handle_ready() {
//     is_printer_ready = true;
//     if (!move_transform) {
//         move_with_transform = [this](std::vector<double> pos, double spd) { this->toolhead->move(pos, spd); };
//         position_with_transform = [this]() { return this->toolhead->get_position(); };
//     }
//     reset_last_position();
// }

void GCodeMove::handle_activate_extruder() {
    reset_last_position();
    extrude_factor = 1.0;
    base_position[3] = last_position[3];
}

void GCodeMove::handle_home_rails_end(const std::vector<int>& axes) {
    reset_last_position();
    for (int axis : axes) {
        base_position[axis] = homing_position[axis];
    }
}

void GCodeMove::set_move_transform(std::shared_ptr<ToolHead> transform, bool force) {
    if (move_transform && !force) {
        throw std::runtime_error("G-Code move transform already specified");
    }
    auto old_transform = move_transform;
    if (!old_transform) {
        old_transform = toolhead;
    }
    move_transform = transform;
    // move_with_transform = [transform](std::vector<double> pos, double spd) { transform->move(pos, spd); };
    // position_with_transform = [transform]() { return transform->get_position(); };
}

void GCodeMove::cmd_G1(const std::unordered_map<std::string, std::string>& params) {
    try {
        for (size_t pos = 0; pos < 3; ++pos) {
            char axis = "XYZ"[pos];
            if (params.find(std::string(1, axis)) != params.end()) {
                double v = std::stod(params.at(std::string(1, axis)));
                if (!absolute_coord) {
                    // value relative to position of last move
                    last_position[pos] += v;
                } else {
                    // value relative to base coordinate position
                    last_position[pos] = v + base_position[pos];
                }
            }
        }

        if (params.find("E") != params.end()) {
            double v = std::stod(params.at("E")) * extrude_factor;
            if (!absolute_coord || !absolute_extrude) {
                // value relative to position of last move
                last_position[3] += v;
            } else {
                // value relative to base coordinate position
                last_position[3] = v + base_position[3];
            }
        }

        if (params.find("F") != params.end()) {
            double gcode_speed = std::stod(params.at("F"));
            if (gcode_speed <= 0.0) {
                throw std::runtime_error("Invalid speed in G1 command");
            }
            speed = gcode_speed * speed_factor;
        }
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error("Unable to parse move in G1 command");
    }

    move_with_transform(last_position, speed);
}

void GCodeMove::reset_last_position() {
    if (is_printer_ready) {
        // last_position = position_with_transform();
    }
}

std::vector<double> GCodeMove::get_gcode_position() {
    std::vector<double> pos(4);
    std::transform(last_position.begin(), last_position.end(), base_position.begin(), pos.begin(), std::minus<double>());
    pos[3] /= extrude_factor;
    return pos;
}

double GCodeMove::get_gcode_speed() {
    return speed / speed_factor;
}

double GCodeMove::get_gcode_speed_override() {
    return speed_factor * 60.0;
}