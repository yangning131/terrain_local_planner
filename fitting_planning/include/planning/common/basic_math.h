#ifndef PLANNING_COMMON_MATH_DATA_H_
#define PLANNING_COMMON_MATH_DATA_H_

#include <math.h>

/**
 * @namespace common
 */
namespace common {

const float kHalfPi = 1.57079632679489662;
const float kPi =     3.14159265358979324;
const float k2Pi =    6.283185307179586477;

/**
 * @brief normalize theta to [0, 2 * pi]
 * @param theta the angle to be normalized
 *
 * @return normalized theta
 */
inline float normalize_angle(float theta) {
    float times = 0.0;
    if(theta > k2Pi) {
        times = floor(theta / k2Pi);
    }
    else if(theta < 0.0) {
        times = ceil(theta / k2Pi);
    }
    else {
        return theta;
    }
    theta += times * k2Pi;
    return theta;
}

}       //namespace common

#endif // PLANNING_COMMON_MATH_DATA_H_
