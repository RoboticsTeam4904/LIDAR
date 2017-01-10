#include <vector>
#include <tuple>

#include "datatypes.h"
#include "doubly_linked_list.h"

#define SLOPE_LIMIT 0.1
#define DISTANCE_LIMIT 512

void blur_points(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
doubly_linked_list_node<line> * get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
