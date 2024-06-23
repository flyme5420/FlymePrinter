#ifndef _TOOLHEAD_H
#define _TOOLHEAD_H

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <functional>
#include <memory>
#include "kinematics/corexy.h"
#include "config.h"
#include "kmove.h"
#include "mcu.h"

extern "C" {
    #include "chelper/trapq.h"
}

// Constants
const double BUFFER_TIME_LOW = 1.0;
const double BUFFER_TIME_HIGH = 2.0;
const double BUFFER_TIME_START = 0.250;
const double BGFLUSH_LOW_TIME = 0.200;
const double BGFLUSH_BATCH_TIME = 0.200;
const double BGFLUSH_EXTRA_TIME = 0.250;
const double MIN_KIN_TIME = 0.100;
const double MOVE_BATCH_TIME = 0.500;
const double STEPCOMPRESS_FLUSH_TIME = 0.050;
const double SDS_CHECK_TIME = 0.001;
const double MOVE_HISTORY_EXPIRE = 30.0;

const double DRIP_SEGMENT_TIME = 0.050;
const double DRIP_TIME = 0.100;
class ToolHead;
class CoreXYKinematics;
class kMove;

using StepGenerator = std::function<void(double)>;

#define LOOKAHEAD_FLUSH_TIME 1

class DripModeEndSignal : public std::exception {};

class LookAheadQueue {
public:
    LookAheadQueue(ToolHead* toolhead);
    void addMove(const kMove& move);
    void flush(bool lazy = false);
    kMove* getLast();
    void setFlushTime(double flush_time);
    void reset();
private:
    ToolHead* toolhead;
    std::vector<kMove> queue;
    double junction_flush;
};


class ToolHead {
public:
    ToolHead(Config& config);
    void kmove(const std::vector<double>& newpos, double speed);
    void register_step_generator(StepGenerator handler);
    void processMoves(const std::vector<kMove>& moves);
    std::vector<double> get_position();
    void set_position(const std::vector<double>& newpos, const std::vector<int>& homing_axes);
    void calcPrintTime();
    void updateDripMoveTime(double next_print_time);
    void advanceMoveTime(double next_print_time);
    void trapqAppend(double next_move_time, const kMove& kmove);
    void advanceFlushTime(double flush_time);
    void noteMCUMovequeueActivity(double mq_time, bool set_step_gen_time = false);

    std::vector<std::shared_ptr<MCU>> all_mcus;
    std::shared_ptr<MCU> mcu;
    std::unique_ptr<LookAheadQueue> lookahead;
    std::vector<double> commanded_pos;
    double max_velocity;
    double max_accel;
    double min_cruise_ratio;
    double square_corner_velocity;
    double junction_deviation;
    double max_accel_to_decel;
    double check_stall_time;
    int print_stall;
    bool can_pause;
    double need_check_pause;
    double print_time;
    std::string special_queuing_state;
    // Timer priming_timer;
    // Timer drip_completion;
    // Timer flush_timer;
    bool do_kick_flush_timer;
    double last_flush_time;
    double min_restart_time;
    double need_flush_time;
    double step_gen_time;
    double clear_history_time;
    double kin_flush_delay;
    // std::shared_ptr<TrapQ> trapq;
    // std::unique_ptr<GCode> gcode;
    // std::unique_ptr<DummyExtruder> extruder;
    std::shared_ptr<CoreXYKinematics> kin;
    struct trapq *trq;
private:
    std::vector<StepGenerator> step_generators;
};

#endif