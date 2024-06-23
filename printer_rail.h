#ifndef PRINTER_RAIL_H
#define PRINTER_RAIL_H

#include <vector>
#include <string>
#include <tuple>
#include <memory>
#include <functional>
#include <unordered_map>
#include "config.h"
// #include "PrinterStepper.h"
// #include "PrinterEndstop.h"
#if 0
class PrinterStepper {
public:
    PrinterStepper(const Config &config, bool units_in_radians) {
        // 初始化步进电机
    }

    std::string get_name(bool short_name = false) const {
        // 返回步进电机的名称
    }

    void setup_itersolve(std::function<void()> alloc_func, ...) {
        // 设置迭代求解
    }

    void generate_steps(float flush_time) {
        // 生成步进指令
    }

    void set_trapq(int trapq) {
        // 设置陷阱队列
    }

    void set_position(float coord) {
        // 设置位置
    }
};

class PrinterEndstop {
public:
    PrinterEndstop(const Config &config) {
        // 初始化限位开关
    }

    bool triggered() const {
        // 检查限位开关是否触发
    }

    void add_stepper(std::shared_ptr<PrinterStepper> stepper) {
        // 添加步进电机
    }

    float get_position_endstop() const {
        // 获取限位开关的位置
    }
};

class PrinterRail {
public:
    PrinterRail(const Config &config, bool need_position_minmax = true, float default_position_endstop = 0.0, bool units_in_radians = false);

    std::tuple<float, float> get_range() const;
    std::tuple<float, float, float, float, bool, float> get_homing_info() const;
    std::vector<std::shared_ptr<PrinterStepper>> get_steppers() const;
    std::vector<std::pair<std::shared_ptr<PrinterEndstop>, std::string>> get_endstops() const;

    // void add_extra_stepper(const Config &config);
    void setup_itersolve(std::function<void()> alloc_func, ...);
    void generate_steps(float flush_time);
    void set_trapq(int trapq);
    void set_position(float coord);

private:
    bool stepper_units_in_radians;
    std::vector<std::shared_ptr<PrinterStepper>> steppers;
    std::vector<std::pair<std::shared_ptr<PrinterEndstop>, std::string>> endstops;
    std::unordered_map<std::string, EndstopInfo> endstop_map;
    float position_endstop;
    float position_min;
    float position_max;
    float homing_speed;
    float second_homing_speed;
    float homing_retract_speed;
    float homing_retract_dist;
    bool homing_positive_dir;

    struct EndstopInfo {
        std::shared_ptr<PrinterEndstop> endstop;
        bool invert;
        bool pullup;
    };

    std::function<std::string()> get_name;
};

PrinterRail LookupMultiRail(const Config &config, bool need_position_minmax = true, float default_position_endstop = 0.0, bool units_in_radians = false);
#endif
#endif // PRINTER_RAIL_H
