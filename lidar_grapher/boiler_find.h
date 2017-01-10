#include <vector>
#include <tuple>

#include "datatypes.h"
#include "doubly_linked_list.h"

#define TARGET_ANGLE M_PI*3.0/4.0
#define MINIMUM_LENGTH 200 // mm
#define MERGE_ANGLE 0.1

boiler_location get_boiler(doubly_linked_list_node<line> * line_data_start);
