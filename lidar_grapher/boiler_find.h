#include <vector>
#include <tuple>

#include "datatypes.h"
#include "doubly_linked_list.h"

/**
   The angle between the boiler and the operator stations
   This is 135*, or 3pi/4 radians
 */
#define TARGET_ANGLE M_PI*3.0/4.0
/**
   The minimum length of a line, in mm.
   Used to reduce uneeded angle calculations
*/
#define MINIMUM_LENGTH 256
/**
   The amount to divide the distance from the origin
   to the endpoint of a line to for comparison to the
   distance between the two endpoints of a line.
   Larger numbers increase sensitivity, smaller numbers
   decrease sensitivity
 */
#define ENDPOINT_DIVISOR 128

/**
   Calculate the location of the boiler based on line data.
   The algorithm determines the angle between all adjacent lines,
   then checks if the end points of the line are near each other.
   Note that this algorithm will fail if multiple 135* angles
   are within lidar vision.
   @param
   	The first node in a doubly linked list of lines.
	This is not modified.
   @return a boiler_location struct containing the RELATIVE location of the boiler
   	Defaults to all zeros if none is found (please check for this)
 */
boiler_location get_boiler(doubly_linked_list_node<line> * line_data_start);
