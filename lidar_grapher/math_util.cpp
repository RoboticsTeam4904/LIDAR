#include "math_util.h"

#include <cmath>

/**
   Calculates the slope of the line between two lidar_datapoints.
   This requires that the cartersian variables of the points be
   calculated already.
*/
float get_slope(lidar_datapoint * point1, lidar_datapoint * point2) {
  float dy = (float) (point2->y - point1->y);
  float dx = (float) (point2->x - point1->x);

  float slope = (dy / dx);

  if (slope > 1 || slope < -1) { // Maintain accuracy at high angles
    slope = (dx / dy);
  }

  return slope;
}

/**
   Calculates the distance between two points.
   Standard Pythagorean theorum. Uses cartersian variables.
*/
float get_distance(lidar_datapoint * point1, lidar_datapoint * point2) {
  float distance = sqrt((float) get_distance_squared(point1, point2));

  return distance;
}

/**
   Calculates the distance between two points.
   Standard Pythagorean theorum. Uses cartersian variables.
*/
int16_t get_distance_squared(lidar_datapoint * point1, lidar_datapoint * point2) {
  int16_t dy = (point2->y - point1->y);
  int16_t dx = (point2->x - point1->x);

  int16_t distance = dx * dx + dy * dy;

  return distance;
}

/**
   Determines if two 16 bit ints are within a certain
   range of each other. range should be positive,
   although this function does not check.
*/
bool in_range(int16_t a, int16_t b, int16_t range) {
  return (a + range > b) && (a - range < b);
}

/**
   Determines if two floats are within a certain
   range of each other. range should be positive,
   although this function does not check.
*/
bool in_range(float a, float b, float range) {
  return (a + range > b) && (a - range < b);
}
