#include <vector>
#include <tuple>

#include "datatypes.h"

void blur_points(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
doubly_linked_list_node<line> * get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
