#ifndef __BOILER_FIND_H__
#define __BOILER_FIND_H__

#include <vector>
#include <tuple>

#include "datatypes.h"
#include "doubly_linked_list.h"

/**
   Constants for alliance
*/
#define BLUE_ALLIANCE 0x01
#define RED_ALLIANCE 0x02
/**
   The angle between the boiler and the operator stations
   This is 135*, or 3pi/4 radians
*/
#define TARGET_ANGLE M_PI/4.0
/**
   Maximum distance between the endpoints in millimeters
*/
#define ENDPOINT_DISTANCE 192
/**
   Range for an angle to be considered the TARGET_ANGLE
*/
#define ANGLE_RANGE M_PI/24.0
/**
 Boiler location adjustments relative to corner
 All dimensions in millimetres
 */
#define BOILER_WIDTH 1066.8f
#define BOILER_DEPTH 456.18f

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
boiler_location get_boiler(doubly_linked_list_node<line> * line_data_start, uint8_t alliance);


#endif
