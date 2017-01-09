#include "datatypes.h"
#include "DoublyLinkedList.h"

#include <vector>
#include <tuple>

int open_teensy(std::string port, int baud = 115200);
DoublyLinkedListNode<LidarDatapoint> * get_lidar_data(int teensy);
int close_teensy(int teensy);
