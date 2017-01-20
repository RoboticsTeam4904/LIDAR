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
   The minimum number of points on a line.
 */
#define MIN_LINE_LENGTH 4

/**
   Calculate the lines within the dataset.
   @param lidar_data_start
   	The "first" element in a doubly linked list of lidar_datapoints
	Note that the list should be circular in both directions
   @return a doubly linked list of lines
 */
doubly_linked_list_node<line> * get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
