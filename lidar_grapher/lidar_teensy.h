#include "datatypes.h"

#include <vector>
#include <tuple>

int open_teensy(std::string port, int baud = 115200);
doubly_linked_list_node<lidar_datapoint> * get_lidar_data(int teensy);
int close_teensy(int teensy);
