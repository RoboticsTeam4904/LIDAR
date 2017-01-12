#include <vector>
#include <tuple>

#include "datatypes.h"
#include "doubly_linked_list.h"

/**
   The algorithm determines the end of a line by looking for a change
   in slope. This is the limit of that change.
 */
#define SLOPE_LIMIT 0.09
/**
   The algorithm also looks for a change in the distance between points.
   This is the limit of that change.
 */
#define DISTANCE_LIMIT 384
/**
   How many times to apply the filter. More is smoother, but tends to
   erode curves and slows calculations.
 */
#define BLUR_COUNT 4
/**
   The minimum number of points on a line.
 */
#define MIN_LINE_LENGTH 4

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
   Calculate the lines within the dataset.
   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoints
	Note that the list should be circular in both directions
   @return a doubly linked list of lines
 */
doubly_linked_list_node<line> * get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
