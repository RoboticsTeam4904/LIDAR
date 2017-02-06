#ifndef __POINT_PREPROCESS_H__
#define __POINT_PREPROCESS_H__

#include "datatypes.h"
#include "doubly_linked_list.h"

/**
   How many times to apply the filter. More is smoother, but tends to
   erode curves and slows calculations.
*/
#define BLUR_COUNT 3

/**
   Threshold for interpolating a point between several other points
*/
#define INTERPOLATE_THRESHOLD 32

/**
   Adds a single point between several other points based on slope
   and distance comparisons. This is performed in place on the
   entire array, rather than just on the linked list of nonzero points.
*/
void interpolate(uint16_t * distances);

/**
   Apply a transform to all the points in a list that "averages" their
   radii with nearby radii. This is similar to a linear approximation
   of a Gaussian blur one point in each direction.
   The transformation is performed in place.
   The cartesian portion of the lidar_datapoint is also filled out at
   the termination of this function.

   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoints
*/
void blur_points(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);

/**
   Add the cartesian portion of each lidar_datapoint

   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoint
*/
void add_cartesians(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);

/**
 Prepare the sin/cosine cache
 */
void init_trig();

#endif
