#ifndef FASTSLAM_UTIL_HPP
#define FASTSLAM_UTIL_HPP

#include <cmath>
#include "as_lib/common.h"

static inline float clamp_angle_pi_pi(float angle) {
    return static_cast<float>(angle - 2 * M_PI * floor((angle + M_PI) / (2 * M_PI)));
}

/**
 * 
 * Simple converter function that takes a string and returns the 
 * corresponding rgb value. 
 * 
 * TODO: add support for different types of orange cones.
 * 
 */

inline uint32_t getColor(int &color){

    uint8_t r = 0, g = 0, b = 0;  

    if (color == YELLOW_CONE){
        r = 255; g = 255;
    }else if (color == BLUE_CONE){
        b = 255;
    }else if (color == ORANGE_CONE){
        r = 255;
    }else if (color == UNKNOWN_CONE){
        r = 200; g = 200; b = 200;
    }

    return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
}

#endif