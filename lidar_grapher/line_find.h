#include <vector>
#include <tuple>

#include "datatypes.h"

typedef std::tuple<LidarDatapoint*, LidarDatapoint*> line;

void blur_points(DoublyLinkedListNode<LidarDatapoint> * lidar_data_start);
std::vector<line> get_lines(DoublyLinkedListNode<LidarDatapoint> * lidar_data_start);
