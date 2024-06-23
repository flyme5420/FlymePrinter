#include "toolhead.h"

LookAheadQueue::LookAheadQueue(ToolHead* toolhead){}

void LookAheadQueue::reset() {
    queue.clear();
    junction_flush = LOOKAHEAD_FLUSH_TIME;
}

void LookAheadQueue::setFlushTime(double flush_time) {
    junction_flush = flush_time;
}

kMove* LookAheadQueue::getLast() {
    if (!queue.empty()) {
        return &queue.back();
    }
    return nullptr;
}

void LookAheadQueue::flush(bool lazy) {
    junction_flush = LOOKAHEAD_FLUSH_TIME;
    bool update_flush_count = lazy;
    size_t flush_count = queue.size();
    std::vector<std::tuple<kMove*, double, double>> delayed;
    double next_end_v2 = 0.0, next_smoothed_v2 = 0.0, peak_cruise_v2 = 0.0;

    for (ssize_t i = flush_count - 1; i >= 0; --i) {
        kMove& kmove = queue[i];
        double reachable_start_v2 = next_end_v2 + kmove.delta_v2;
        double start_v2 = std::min(kmove.max_start_v2, reachable_start_v2);
        double reachable_smoothed_v2 = next_smoothed_v2 + kmove.smooth_delta_v2;
        double smoothed_v2 = std::min(kmove.max_smoothed_v2, reachable_smoothed_v2);

        if (smoothed_v2 < reachable_smoothed_v2) {
            if (smoothed_v2 + kmove.smooth_delta_v2 > next_smoothed_v2 || !delayed.empty()) {
                if (update_flush_count && peak_cruise_v2) {
                    flush_count = i;
                    update_flush_count = false;
                }
                peak_cruise_v2 = std::min(kmove.max_cruise_v2, (smoothed_v2 + reachable_smoothed_v2) * 0.5);
                if (!delayed.empty()) {
                    if (!update_flush_count && i < flush_count) {
                        double mc_v2 = peak_cruise_v2;
                        for (auto& [m, ms_v2, me_v2] : delayed) {
                            mc_v2 = std::min(mc_v2, ms_v2);
                            m->setJunction(std::min(ms_v2, mc_v2), mc_v2, std::min(me_v2, mc_v2));
                        }
                        delayed.clear();
                    }
                }
            }
            if (!update_flush_count && i < flush_count) {
                double cruise_v2 = std::min({(start_v2 + reachable_start_v2) * 0.5, kmove.max_cruise_v2, peak_cruise_v2});
                kmove.setJunction(std::min(start_v2, cruise_v2), cruise_v2, std::min(next_end_v2, cruise_v2));
            }
        } else {
            delayed.emplace_back(&kmove, start_v2, next_end_v2);
            
        }
        next_end_v2 = start_v2;
        next_smoothed_v2 = smoothed_v2;
    }

    if (update_flush_count || flush_count == 0) {
        return;
    }

    auto it = queue.begin();
    auto end = queue.begin();
    std::advance(end, flush_count);

    std::vector<kMove> moves;
    moves.insert(moves.end(), it, end);

    toolhead->processMoves(moves);
    queue.erase(queue.begin(), queue.begin() + flush_count);
}

void LookAheadQueue::addMove(const kMove& kmove) {
    queue.push_back(kmove);
    if (queue.size() == 1) {
        return;
    }
    queue.back().calcJunction(queue[queue.size() - 2]);
    junction_flush -= kmove.min_move_t;
    if (junction_flush <= 0.0) {
        flush(true);
    }
}

ToolHead::ToolHead(Config &config) {
    // printer = config.getPrinter();
    // reactor = printer.getReactor();
    // all_mcus = printer.lookupObjects<MCU>("mcu");
    mcu = all_mcus[0];
    lookahead = std::make_unique<LookAheadQueue>(this);
    lookahead->setFlushTime(BUFFER_TIME_HIGH);
    commanded_pos = {0.0, 0.0, 0.0, 0.0};
    
    // Velocity and acceleration control
    max_velocity = config.max_velocity;
    max_accel = config.max_accel;
    double min_cruise_ratio = 0.5;
    if (!config.minimum_cruise_ratio) {
        double req_accel_to_decel = config.max_accel_to_decel;
        if (req_accel_to_decel != 0.0) {
            // config.deprecate("max_accel_to_decel");
            min_cruise_ratio = 1.0 - std::min(1.0, req_accel_to_decel / max_accel);
        }
    }
    min_cruise_ratio = config.minimum_cruise_ratio;
    square_corner_velocity = config.square_corner_velocity;
    junction_deviation = max_accel_to_decel = 0.0;
    // calcJunctionDeviation();
    
    // Input stall detection
    check_stall_time = 0.0;
    print_stall = 0;
    // Input pause tracking
    can_pause = false; //!mcu->isFileOutput();
    need_check_pause = -1.0;
    // Print time tracking
    print_time = 0.0;
    special_queuing_state = "NeedPrime";
    // priming_timer = nullptr;
    // drip_completion = nullptr;
    
    // Flush tracking
    // flush_timer = reactor->registerTimer([this](double eventtime) { return flushHandler(eventtime); });
    do_kick_flush_timer = true;
    last_flush_time = min_restart_time = 0.0;
    need_flush_time = step_gen_time = clear_history_time = 0.0;
    
    // Kinematic step generation scan window time tracking
    kin_flush_delay = SDS_CHECK_TIME;
    
    // Setup iterative solver
    trq = trapq_alloc();
    
    // Create kinematics class
    // gcode = printer.lookupObject<GCode>("gcode");
    // extruder = std::make_unique<DummyExtruder>(printer);
    // std::string kin_name = config.getString("kinematics");
    // try {
    //     kin = KinematicsFactory::loadKinematics(this, config, kin_name);
    // } catch (const std::exception& e) {
    //     throw ConfigError("Error loading kinematics '" + kin_name + "'");
    // }
    
}

void ToolHead::register_step_generator(StepGenerator handler) {
    step_generators.push_back(handler);
}

void ToolHead::processMoves(const std::vector<kMove>& moves) {
    // Resync print_time if necessary
    if (!special_queuing_state.empty()) {
        if (special_queuing_state != "Drip") {
            // Transition from "NeedPrime"/"Priming" state to main state
            special_queuing_state = "";
            need_check_pause = -1.0;
        }
        // calcPrintTime();
    }

    // Queue moves into trapezoid motion queue (trapq)
    double next_move_time = print_time;
    for (const auto& kmove : moves) {
        if (kmove.is_kinematic_move) {
            trapq_append(
                trq, next_move_time,
                kmove.accel_t, kmove.cruise_t, kmove.decel_t,
                kmove.start_pos[0], kmove.start_pos[1], kmove.start_pos[2],
                kmove.axes_r[0], kmove.axes_r[1], kmove.axes_r[2],
                kmove.start_v, kmove.cruise_v, kmove.accel);
        }
        // if (kmove.axes_d[3]) {
        //     extruder.kmove(next_move_time, kmove);
        // }
        next_move_time = next_move_time + kmove.accel_t + kmove.cruise_t + kmove.decel_t;
        // for (const auto& cb : kmove.timing_callbacks) {
        //     cb(next_move_time);
        // }
    }

    // Generate steps for moves
    if (special_queuing_state != "") {
        updateDripMoveTime(next_move_time);
    }
    // noteMCUMovequeueActivity(next_move_time + kin_flush_delay, true);
    advanceMoveTime(next_move_time);
}

void ToolHead::calcPrintTime() {
    // double curtime = reactor.monotonic();
    // double est_print_time = mcu.estimatedPrintTime(curtime);
    // double kin_time = std::max(est_print_time + MIN_KIN_TIME, min_restart_time);
    // kin_time += kin_flush_delay;
    // double min_print_time = std::max(est_print_time + BUFFER_TIME_START, kin_time);
    // if (min_print_time > print_time) {
    //     print_time = min_print_time;
        // printer.sendEvent("toolhead:sync_print_time", curtime, est_print_time, print_time);
    // }
}

void ToolHead::advanceMoveTime(double next_print_time) {
    double pt_delay = kin_flush_delay + STEPCOMPRESS_FLUSH_TIME;
    double flush_time = std::max(last_flush_time, print_time - pt_delay);
    print_time = std::max(print_time, next_print_time);
    double want_flush_time = std::max(flush_time, print_time - pt_delay);
    while (true) {
        flush_time = std::min(flush_time + MOVE_BATCH_TIME, want_flush_time);
        advanceFlushTime(flush_time);
        if (flush_time >= want_flush_time) {
            break;
        }
    }
}

void ToolHead::updateDripMoveTime(double next_print_time) {
    double flush_delay = DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + kin_flush_delay;
    while (print_time < next_print_time) {
        // if (drip_completion.test()) {
        //     throw DripModeEndSignal();
        // }
        // double curtime = reactor.monotonic();
        // double est_print_time = mcu.estimatedPrintTime(curtime);
        // double wait_time = print_time - est_print_time - flush_delay;
        // if (wait_time > 0.0 && can_pause) {
        //     // Pause before sending more steps
        //     drip_completion.wait(curtime + wait_time);
        //     continue;
        // }
        double npt = std::min(print_time + DRIP_SEGMENT_TIME, next_print_time);
        noteMCUMovequeueActivity(npt + kin_flush_delay, true);
        advanceMoveTime(npt);
    }
}

void ToolHead::noteMCUMovequeueActivity(double mq_time, bool set_step_gen_time) {
    need_flush_time = std::max(need_flush_time, mq_time);
    if (set_step_gen_time) {
        step_gen_time = std::max(step_gen_time, mq_time);
    }
    if (do_kick_flush_timer) {
        do_kick_flush_timer = false;
        // reactor.updateTimer(flush_timer, reactor.NOW);
    }
}

// Helper functions and member variables
void ToolHead::trapqAppend(double next_move_time, const kMove& kmove) {
    // Implement the function to append kmove to trapezoid motion queue
}

void ToolHead::advanceFlushTime(double flush_time) {
    // Implement the function to advance flush time
    flush_time = std::max(flush_time, this->last_flush_time);
    // Generate steps via itersolve
    double sg_flush_want = std::min(flush_time + STEPCOMPRESS_FLUSH_TIME,
                                    this->print_time - this->kin_flush_delay);
    double sg_flush_time = std::max(sg_flush_want, flush_time);
    for (auto& sg : this->step_generators) {
        sg(sg_flush_time);
    }
    this->min_restart_time = std::max(this->min_restart_time, sg_flush_time);
    // Free trapq entries that are no longer needed
    double clear_history_time = this->clear_history_time;
    if (!this->can_pause) {
        clear_history_time = flush_time - MOVE_HISTORY_EXPIRE;
    }
    double free_time = sg_flush_time - this->kin_flush_delay;
    trapq_finalize_moves(this->trq, free_time, clear_history_time);   //trapq.c
    // this->extruder.update_move_time(free_time, clear_history_time);
    // Flush stepcompress and mcu steppersync
    for (auto& m : this->all_mcus) {
        m->flush_moves(flush_time, clear_history_time);
    }
    this->last_flush_time = flush_time;
}

std::vector<double> ToolHead::get_position() {
    return commanded_pos;
}

void ToolHead::set_position(const std::vector<double>& newpos, const std::vector<int>& homing_axes) {
    // flush_step_generation();
    trapq_set_position(trq, print_time, newpos[0], newpos[1], newpos[2]);
    commanded_pos = newpos;
    // 设定位置和 homing_axes
    // kin->set_position(newpos, homing_axes);
    // send_event("toolhead:set_position");
}

void ToolHead::kmove(const std::vector<double>& newpos, double speed) {
    kMove kmove(this, commanded_pos, newpos, speed);
    if (kmove.move_d == 0) {
        return;
    }
    if (kmove.is_kinematic_move) {
        // 进行运动学检查
        kin->check_move(kmove);
    }
    if (kmove.axes_d[3] != 0) {
        // 进行挤出机检查
        // extruder.check_move(kmove);
    }
    commanded_pos = kmove.end_pos;
    // 添加到前瞻队列
    lookahead->addMove(kmove);
    if (print_time > need_check_pause) {
        // _check_pause();
    }
}


    // Register the generate_steps function as a step generator
    // toolhead.register_step_generator([&stepper](double flush_time) {
    //     stepper.generate_steps(flush_time);
    // });