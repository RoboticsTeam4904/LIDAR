#include "datatypes.h"
#include "doubly_linked_list.h"

#include <vector>
#include <tuple>

/**
   Connect to teensy at dev port port with baud rate baud
 */
int open_teensy(std::string port, int baud = 115200);
/**
   Read data from teensy as a doubly linked list
   Note that no blurring has occured yet
   @param teensy should be the int returned from open_teensy
 */
doubly_linked_list_node<lidar_datapoint> * get_lidar_data(int teensy);
/**
   Cleanup teensy
   @param teensy should be the int return from open_teensy
 */
int close_teensy(int teensy);
