#include <vector>
#include <tuple>

#include "datatypes.h"

void blur_points(DoublyLinkedListNode<LidarDatapoint> * lidar_data_start);
DoublyLinkedListNode<line> * get_lines(DoublyLinkedListNode<LidarDatapoint> * lidar_data_start);
