#include <algorithm> // For std::max
#include <cstdint>   // For int64_t
#include "mcu.h"

extern "C" {
    #include "chelper/stepcompress.h"
}

void MCU::flush_moves(double print_time, double clear_history_time) {
    if (this->_steppersync == nullptr) {
        return;
    }
    int64_t clock = print_time_to_clock(print_time);
    if (clock < 0) {
        return;
    }
    for (auto& cb : this->_flush_callbacks) {
        cb(print_time, clock);
    }
    int64_t clear_history_clock = std::max(static_cast<int64_t>(0), print_time_to_clock(clear_history_time));
    int ret = steppersync_flush((struct steppersync *)this->_steppersync, clock, clear_history_clock);
    if (ret) {
        throw std::runtime_error("Internal error in MCU '" + this->_name + "' stepcompress");
    }
}

int64_t MCU::print_time_to_clock(double print_time) {
    return 0; //this->_clocksync.print_time_to_clock(print_time);
}

