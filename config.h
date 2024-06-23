#ifndef _CONFIG_H
#define _CONFIG_H

#include <iostream>
#include <vector>
#include <string>

class Config {
public:
    Config();
    float max_velocity;
    float max_accel;
    float minimum_cruise_ratio;
    float max_accel_to_decel;
    float square_corner_velocity;

};

#endif