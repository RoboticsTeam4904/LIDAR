#ifndef __MATH_UTIL_H__
#define __MATH_UTIL_H__

#include "datatypes.h"

/**
   Calculates the slope of the line between two lidar_datapoints.
   This requires that the cartersian variables of the points be
   calculated already.
 */
float get_slope(lidar_datapoint * point1, lidar_datapoint * point2);

/**
   Calculates the distance between two points.
   Standard Pythagorean theorum. Uses cartersian variables.
 */
float get_distance(lidar_datapoint * point1, lidar_datapoint * point2);

/**
   Determines if two 16 bit ints are within a certain
   range of each other. range should be positive,
   although this function does not check.
 */
bool in_range(int16_t a, int16_t b, int16_t range);

/**
   Determines if two floats are within a certain
   range of each other. range should be positive,
   although this function does not check.
 */
bool in_range(float a, float b, float range);

#endif
