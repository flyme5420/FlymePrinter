#ifndef _MCU_H
#define _MCU_H

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <cstdint>

class MCU {
public:
    void flush_moves(double print_time, double clear_history_time);
    int64_t print_time_to_clock(double print_time);

private:
    void* _steppersync = nullptr; // Assuming _steppersync is a pointer to some structure
    std::vector<std::function<void(double, int64_t)>> _flush_callbacks;
    std::string _name;
    // Clocksync _clocksync; // Assuming _clocksync is an instance of a class named Clocksync
    // FFILib _ffi_lib; // Assuming _ffi_lib is an instance of a class named FFILib
};

#endif