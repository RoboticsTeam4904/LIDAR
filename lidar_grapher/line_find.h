#include <vector>
#include <tuple>

#include "datatypes.h"

typedef std::tuple<lidar_datapoint*, lidar_datapoint*> line;

void blur_points(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
std::vector<line> get_lines(doubly_linked_list_node<lidar_datapoint> * lidar_data_start);
