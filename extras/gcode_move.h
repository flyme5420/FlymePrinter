#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include "../toolhead.h"

class GCodeMove {
public:
    GCodeMove(std::shared_ptr<ToolHead> toolhead);
    void cmd_G1(const std::unordered_map<std::string, std::string>& params);
    void handle_activate_extruder();
    void handle_home_rails_end(const std::vector<int>& axes);
    void set_move_transform(std::shared_ptr<ToolHead> transform, bool force = false);
    double get_gcode_speed_override();
    double get_gcode_speed();
    std::vector<double> get_gcode_position();
    void reset_last_position();
private:
    bool absolute_coord;
    bool absolute_extrude;
    std::vector<double> base_position;
    std::vector<double> last_position;
    std::vector<double> homing_position;
    double speed;
    double speed_factor;
    double extrude_factor;
    std::unordered_map<std::string, std::vector<double>> saved_states;
    std::shared_ptr<ToolHead> move_transform;
    std::function<void(std::vector<double>, double)> move_with_transform;
    std::function<std::vector<double>()> position_with_transform;
    bool is_printer_ready = false;
    std::shared_ptr<ToolHead> toolhead;
};


