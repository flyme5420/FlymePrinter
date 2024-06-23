#include <iostream>
#include <unordered_map>
#include <string>

#include "extras/gcode_move.h"
#include "toolhead.h"
#include "stepper.h"
#include "config.h"

int main() {

    std::unordered_map<std::string, std::string> params = {
        {"X", "10.5"},
        {"Y", "20.0"},
        {"Z", "5.0"},
        {"F", "1500"}
    };

    MCU_stepper mcu_stepper("", 1, 0.2);
    Config config;
    std::shared_ptr<ToolHead> toolhead = std::make_shared<ToolHead>(config);
    StepGenerator step_gen = [&mcu_stepper](double value) { mcu_stepper.generate_steps(value); };
    toolhead->register_step_generator(step_gen);
    GCodeMove gcode_move(toolhead);
    gcode_move.cmd_G1(params);

    return 0;
}